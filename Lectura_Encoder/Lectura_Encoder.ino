// === Pines del Encoder ===
#define ENCODER_PIN_A 2  // INT0
#define ENCODER_PIN_B 3  // INT1

// === Pines del L298N ===
#define MOTOR_PWM 8
#define MOTOR_IN1 10
#define MOTOR_IN2 9

volatile long encoderPos = 0;

void setup() {
  Serial.begin(9600);

  // === Configuración del encoder ===
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), handleEncoderB, CHANGE);

  // === Configuración del motor ===
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // Iniciar motor en sentido horario con PWM máximo (255)
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 255);  // PWM máximo
}

void loop() {
  static unsigned long lastPrint = 0;
  static long lastPos = 0;

  if (millis() - lastPrint >= 500) {
    long pos;
    noInterrupts();
    pos = encoderPos;
    interrupts();

    long delta = pos - lastPos;
    float rpm = (delta / 3300.0) * (60000.0 / 500.0);  // PPR = 3300
    float rpm = rpm*(130/155.91);
    Serial.print("Pulsos: ");
    Serial.print(pos);
    Serial.print(" | RPM: ");
    Serial.println(rpm);

    lastPos = pos;
    lastPrint = millis();
  }
}

// === RUTINAS DE INTERRUPCIÓN ===

void handleEncoderA() {
  bool A = digitalRead(ENCODER_PIN_A);
  bool B = digitalRead(ENCODER_PIN_B);
  if (A == B) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

void handleEncoderB() {
  bool A = digitalRead(ENCODER_PIN_A);
  bool B = digitalRead(ENCODER_PIN_B);
  if (A != B) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}
