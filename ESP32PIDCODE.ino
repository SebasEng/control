#define ENCA 12 // YELLOW Digital inputs
#define ENCB 13 // BLUE
#define IN3 32
#define IN4 33
#define PWMPin 25
#define CALIBRATION_BTN 14
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
int lastButtonState = HIGH;
int buttonState = HIGH;


// Variables globales
volatile int pos = 0;       // Posición relativa del encoder
int posOffset = 0;          // Offset para mantener referencia absoluta
long prevT = 0; 
float eprev = 0;
float eintegral = 0;
volatile int target;

// Control de velocidad de muestreo
#define SAMPLE_TIME 20 // ms entre iteraciones del PID

// Variables para limitar la velocidad de cambio del setpoint
int prevTarget = 0;
#define MAX_TARGET_CHANGE 10 // máxima variación permitida por ciclo

// Opciones configurables - podrías implementar un menú por serial
float kp = 15.89; 
float kd = 0.23;
float ki = 1.00;
float maxIntegral = 100.0; // Límite anti-windup

// Variables para datos
unsigned long startTime = 0;
float elapsedTime = 0;
unsigned long lastSampleTime = 0;

// Referencia absoluta
bool controlAbsoluto = true;     // Control en modo relativo o absoluto
int posicionAbsoluta = 0;         // Posición absoluta a mantener

void setup(){
  Serial.begin(115200); // Mayor velocidad para evitar bloqueos
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  

  // Configurar motor
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWMPin, OUTPUT);
  pinMode(CALIBRATION_BTN, INPUT_PULLUP);
  
  // Imprimir cabecera CSV
  Serial.println("tiempo,target,posicion,error,pwm,kp,ki,kd,integral,derivada,posAbs");


  
  // Iniciar el contador de tiempo
  startTime = millis();
  lastSampleTime = startTime;
  
  // Mensaje inicial
  Serial.println("Sistema PID iniciado");
  Serial.println("Comandos disponibles:");
  Serial.println("p[valor] : Cambiar Kp");
  Serial.println("i[valor] : Cambiar Ki");
  Serial.println("d[valor] : Cambiar Kd");
  Serial.println("z        : Establecer posición actual como cero absoluto");
  Serial.println("m[valor] : Ir a posición absoluta [valor]");
  Serial.println("r        : Reset integral");
  Serial.println("a        : Alternar entre control relativo/absoluto");

  controlAbsoluto = true;
}

void loop(){
  // Control de la frecuencia de muestreo
  unsigned long currentMillis = millis();
  if (currentMillis - lastSampleTime < SAMPLE_TIME) {
    return; // Salir si no ha pasado suficiente tiempo
  }
  lastSampleTime = currentMillis;
  checkCalibrationButton();
  
  // Calcular tiempo transcurrido
  elapsedTime = (currentMillis - startTime) / 1000.0; // en segundos
  
  // Calcular posición absoluta
  int posAbs = pos + posOffset;
  
  // Determinar el target según el modo de control
  
  
  target = posicionAbsoluta - posOffset;  // Convertir posición absoluta a relativa
  // Cálculo de PID
  long currT = micros();
  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT;
  
  // Error actual
  int e = pos-target;

  // Cálculo del componente derivativo
  float dedt = (e-eprev)/(deltaT);
  
  // Cálculo del componente integral con anti-windup
  if (abs(e) > 2) {
    eintegral = eintegral + e*deltaT;
  }
  
  // Limitar el valor integral para evitar saturación (anti-windup)
  if(eintegral > maxIntegral) eintegral = maxIntegral;
  if(eintegral < -maxIntegral) eintegral = -maxIntegral;

  // Control signal con los tres componentes
  float pTerm = kp * e;
  float iTerm = ki * eintegral;
  float dTerm = kd * dedt;
  
  float u = pTerm + iTerm + dTerm;

  // Limitar la potencia de salida
  float pwr = fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  
  // Zona muerta del motor (ajustable)
  float deadband = 30;
  if(pwr > 0 && pwr < deadband){
    pwr = deadband;
  } 
  
  // Si el error es muy pequeño, detener el motor
  if(abs(e) < 2){
    pwr = 0;
  }
  
  // Determinar dirección
  int dir = 1; 
  if(u < 0){
    dir = -1;
  }
  if(pwr == 0){
    dir = 0;
  }

  // Accionar el motor
  setMotor(dir, pwr, PWMPin, IN3, IN4);
  
  // Guardar error para el siguiente ciclo
  eprev = e;
  
  // Imprimir datos en formato CSV
  Serial.print(elapsedTime, 3); // Tiempo con 3 decimales
  Serial.print(",");
  Serial.print(posicionAbsoluta);
  Serial.print(",");
  Serial.print(posAbs);
  Serial.print(",");
  Serial.print(e);
  Serial.print(",");
  Serial.print(pwr);
  Serial.print(",");
  Serial.print(kp, 3);
  Serial.print(",");
  Serial.print(ki, 3);
  Serial.print(",");
  Serial.print(kd, 3);
  Serial.print(",");
  Serial.print(eintegral, 3);
  Serial.print(",");
  Serial.print(dedt, 3);
  Serial.print(",");
  Serial.println(posAbs);  // Posición absoluta
  
  // Verificar si hay comandos por serial
  checkSerialCommand();
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b>0){
    pos++;
  }
  else{
    pos--;
  }
}

void setMotor(int dir, int pwmVal, int pwm, int in3, int in4){
  analogWrite(pwm, pwmVal); // set speed
  if(dir == 1){
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if(dir == -1){
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else{
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
}

// Función para establecer la posición actual como cero absoluto
void setCeroAbsoluto() {
  posOffset = -pos;  // El offset es lo que hay que sumar a pos para obtener 0
  posicionAbsoluta = 0;  // Iniciar en cero la posición absoluta
  eintegral = 0;     // Reset del término integral
  controlAbsoluto = true;  // Cambiar a modo absoluto
  Serial.println("Posición actual establecida como cero absoluto");
  Serial.println("Modo de control cambiado a absoluto");

}

// Función para cambiar parámetros del PID en tiempo real
void checkSerialCommand() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Comandos sin valor numérico
    if (cmd == 'z') {
      setCeroAbsoluto();
      return;
    } else if (cmd == 'r') {
      eintegral = 0;
      Serial.println("Integral reseteada");
      return;
    }
    
    // Comandos con valor numérico
    float value = 0;
    
    // Esperar a que lleguen más datos
    delay(10);
    if (Serial.available() > 0) {
      value = Serial.parseFloat();
      
      switch(cmd) {
        case 'p': // Cambiar Kp
          kp = value;
          Serial.print("Nuevo Kp: ");
          Serial.println(kp, 3);
          break;
        case 'i': // Cambiar Ki
          ki = value;
          Serial.print("Nuevo Ki: ");
          Serial.println(ki, 3);
          break;
        case 'd': // Cambiar Kd
          kd = value;
          Serial.print("Nuevo Kd: ");
          Serial.println(kd, 3);
          break;
        case 'm': // Ir a posición absoluta
          posicionAbsoluta = value;
          controlAbsoluto = true;  // Cambiar a modo absoluto
          Serial.print("Moviendo a posición absoluta: ");
          Serial.println(posicionAbsoluta);
          break;
        default:
          break;
      }
    }
  }
}

// Nueva función para manejar el botón de calibración con debounce
void checkCalibrationButton() {
  // Leer el estado actual del botón
  int reading = digitalRead(CALIBRATION_BTN);
  
  // Verificar si ha cambiado
  if (reading != lastButtonState) {
    // Reiniciar el timer de debounce
    lastDebounceTime = millis();
  }
  
  // Verificar si ha pasado suficiente tiempo
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Si el estado es diferente al último estado registrado
    if (reading != buttonState) {
      buttonState = reading;
      
      // Si el botón ha sido presionado (LOW en pullup)
      if (buttonState == LOW) {
        // Realizar calibración
        setCeroAbsoluto();
      }
    }
  }
  
  // Guardar el estado para la próxima iteración
  lastButtonState = reading;
}