//
//==================================================
// MOTOR DERECHO
//==================================================
//

#define PWMB 26
#define BIN1 27
#define BIN2 14
#define STBY 13

//
//==================================================
// ENCODER
//==================================================
//

#define ENCODER_A 34
#define ENCODER_B 35

//
//==================================================
// PWM CONFIG
//==================================================
//

#define PWM_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

//
//==================================================
// CONSTANTES
//==================================================
//

const float PPR = 4230.0;
const float SAMPLE_TIME = 0.1;

//
//==================================================
// ENCODER
//==================================================
//

volatile long encoderPos = 0;

//
//==================================================
// RPM
//==================================================
//

float rpm_raw = 0;
float rpm_ema = 0;

float alpha = 0.197;

//
//==================================================
// PID
//==================================================
//

float setpoint = 100.0;

float kp = 2.52522;
float ki = 2.0829;
float kd = 1.415;

float error = 0;
float prev_error = 0;
float integral = 0;

//
//==================================================
// PWM
//==================================================
//

int pwm_output = 0;

//
//==================================================
// TIEMPO
//==================================================
//

unsigned long tiempo_inicio = 0;

//
//==================================================
// SETUP
//==================================================
//

void setup() {

  Serial.begin(115200);

  setupMotor();

  setupEncoder();

  tiempo_inicio = millis();

  Serial.println("SYSTEM READY");
}

//
//==================================================
// LOOP
//==================================================
//

void loop() {

  static unsigned long lastTime = 0;

  if (millis() - lastTime >= 100) {

    updateRPM();

    applyEMAFilter();

    if (millis() - tiempo_inicio < 1000) {

      applyStartupPWM();

    } else {

      updatePID();

      applyPWM();
    }

    printData();

    lastTime = millis();
  }
}

//
//==================================================
// SETUP MOTOR
//==================================================
//

void setupMotor() {

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  //--------------------------------
  // DIRECCION
  //--------------------------------

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  //--------------------------------
  // PWM
  //--------------------------------

  ledcSetup(
      PWM_CHANNEL,
      PWM_FREQ,
      PWM_RESOLUTION);

  ledcAttachPin(
      PWMB,
      PWM_CHANNEL);
}

//
//==================================================
// SETUP ENCODER
//==================================================
//

void setupEncoder() {

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_A),
      handleEncoderA,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_B),
      handleEncoderB,
      CHANGE);
}

//
//==================================================
// UPDATE RPM
//==================================================
//

void updateRPM() {

  static long lastPos = 0;

  long pos;

  noInterrupts();
  pos = encoderPos;
  interrupts();

  long delta = pos - lastPos;

  rpm_raw =
      (delta / PPR) *
      (60000.0 / 100.0);

  lastPos = pos;
}

//
//==================================================
// EMA FILTER
//==================================================
//

void applyEMAFilter() {

  rpm_ema =
      alpha * rpm_raw +
      (1.0 - alpha) * rpm_ema;
}

//
//==================================================
// STARTUP PWM
//==================================================
//

void applyStartupPWM() {

  int pwm_directo =
      (setpoint * 225.0) / 130.0;

  if (pwm_directo > 225)
    pwm_directo = 225;

  ledcWrite(
      PWM_CHANNEL,
      pwm_directo);

  pwm_output = pwm_directo;
}

//
//==================================================
// PID
//==================================================
//

void updatePID() {

  //--------------------------------
  // ERROR
  //--------------------------------

  error =
      setpoint - rpm_ema;

  //--------------------------------
  // INTEGRAL
  //--------------------------------

  integral +=
      error * SAMPLE_TIME;

  //--------------------------------
  // DERIVATIVE
  //--------------------------------

  float derivative =
      (error - prev_error) /
      SAMPLE_TIME;

  //--------------------------------
  // OUTPUT
  //--------------------------------

  float output =
      kp * error +
      ki * integral +
      kd * derivative;

  //--------------------------------
  // PWM INCREMENTAL
  //--------------------------------

  pwm_output +=
      (int)output;

  //--------------------------------
  // LIMITES
  //--------------------------------

  if (pwm_output > 255)
    pwm_output = 255;

  if (pwm_output < 0)
    pwm_output = 0;

  //--------------------------------
  // SAVE
  //--------------------------------

  prev_error = error;
}

//
//==================================================
// APPLY PWM
//==================================================
//

void applyPWM() {

  ledcWrite(
      PWM_CHANNEL,
      pwm_output);
}

//
//==================================================
// SERIAL
//==================================================
//

void printData() {

  Serial.print("RPM RAW: ");
  Serial.print(rpm_raw);

  Serial.print(" | RPM EMA: ");
  Serial.print(rpm_ema);

  Serial.print(" | PWM: ");
  Serial.print(pwm_output);

  Serial.print(" | ERROR: ");
  Serial.println(error);
}

//
//==================================================
// ENCODER ISR
//==================================================
//

void handleEncoderA() {

  bool A = digitalRead(ENCODER_A);
  bool B = digitalRead(ENCODER_B);

  if (A == B)
    encoderPos++;
  else
    encoderPos--;
}

void handleEncoderB() {

  bool A = digitalRead(ENCODER_A);
  bool B = digitalRead(ENCODER_B);

  if (A != B)
    encoderPos++;
  else
    encoderPos--;
}