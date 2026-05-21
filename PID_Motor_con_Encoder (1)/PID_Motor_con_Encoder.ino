// === Pines del Encoder ===
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3

// === Pines del L298N ===
#define MOTOR_PWM 8
#define MOTOR_IN1 10
#define MOTOR_IN2 9

// === Variables globales ===
volatile long encoderPos = 0;

float setpoint = 100.0;  // RPM deseado
float kp =	3.12522, ki = 2.06929, kd = 1.405;
float error = 0, prev_error = 0, integral = 0;

int pwm_output = 0;  // PWM actual (0–255)

float alpha = 0.197;    // Factor de suavizado EMA
float rpm_ema = 0;      // Valor suavizado de RPM

unsigned long tiempo_inicio = 0;

// === Setup ===
void setup() {
  Serial.begin(9600);

  // Encoder
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), handleEncoderB, CHANGE);

  // Motor
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // Sentido horario por defecto
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);

  // Guardar el tiempo de inicio
  tiempo_inicio = millis();
}

// === Loop principal ===
void loop() {
  static unsigned long lastTime = 0;
  static long lastPos = 0;

  if (millis() - lastTime >= 100) {  // cada 100 ms
    long pos;
    noInterrupts();
    pos = encoderPos;
    interrupts();

    long delta = pos - lastPos;
    float rpm_real = (delta / 3300.0) * 600.0;  // RPM bruta
    float rpm = rpm_real * (130.0 / 155.91);    // Ajuste por regla de tres
    rpm_ema = alpha * rpm + (1 - alpha) * rpm_ema;

    if (millis() - tiempo_inicio < 1000) {
      // 🚫 PRIMER SEGUNDO: SIN PID, solo regla de tres
      int pwm_directo = (setpoint * 225) / 130;  // regla de tres
      if (pwm_directo > 225) pwm_directo = 225;
      analogWrite(MOTOR_PWM, pwm_directo);

      Serial.print("PWM REGLA 3: ");
      Serial.println(pwm_directo);
    } else {
      // ✅ DESPUÉS DE 1 SEGUNDO: ACTIVAR PID
      error = setpoint - rpm_ema;
      integral += error * 0.1;
      float derivative = (error - prev_error) / 0.1;
      float output = kp * error + ki * integral + kd * derivative;
      prev_error = error;

      pwm_output += (int)output;
      if (pwm_output > 255) pwm_output = 255;
      if (pwm_output < 0) pwm_output = 0;

      analogWrite(MOTOR_PWM, pwm_output);

      Serial.print("RPM cruda: ");
      Serial.print(rpm);
      Serial.print(" | EMA: ");
      Serial.print(rpm_ema);
      Serial.print(" | PWM: ");
      Serial.print(pwm_output);
      Serial.print(" | Error: ");
      Serial.println(error);
    }

    lastPos = pos;
    lastTime = millis();
  }
}

// === Rutinas de interrupción ===
void handleEncoderA() {
  bool A = digitalRead(ENCODER_PIN_A);
  bool B = digitalRead(ENCODER_PIN_B);
  encoderPos += (A == B) ? 1 : -1;
}

void handleEncoderB() {
  bool A = digitalRead(ENCODER_PIN_A);
  bool B = digitalRead(ENCODER_PIN_B);
  encoderPos += (A != B) ? 1 : -1;
}
