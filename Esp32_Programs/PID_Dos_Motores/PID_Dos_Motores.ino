//
//==================================================
// MOTOR DERECHO
//==================================================
//

#define PWMB 26
#define BIN1 27
#define BIN2 14

//
//==================================================
// MOTOR IZQUIERDO
//==================================================
//

#define PWMA 32
#define AIN1 33
#define AIN2 25

//
//==================================================
// STBY
//==================================================
//

#define STBY 13

//
//==================================================
// ENCODERS
//==================================================
//

#define ENCODER_RIGHT_A 34
#define ENCODER_RIGHT_B 35

#define ENCODER_LEFT_A 39
#define ENCODER_LEFT_B 36

//
//==================================================
// PWM
//==================================================
//

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

#define PWM_CHANNEL_RIGHT 0
#define PWM_CHANNEL_LEFT 1

//
//==================================================
// CONSTANTES
//==================================================
//

const float PPR = 4230.0;
const float SAMPLE_TIME = 0.1;

//
//==================================================
// ENCODERS
//==================================================
//

volatile long encoderRightPos = 0;
volatile long encoderLeftPos = 0;

//
//==================================================
// SETPOINTS
//==================================================
//

float setpointRight = 50.0;
float setpointLeft = 80.0;

//
//==================================================
// PID RIGHT
//==================================================
//

float kpR = 2.52522;
float kiR = 2.0829;
float kdR = 1.305;

float errorR = 0;
float prev_errorR = 0;
float integralR = 0;

//
//==================================================
// PID LEFT
//==================================================
//

float kpL = 2.52522;
float kiL = 2.0829;
float kdL = 1.305;

float errorL = 0;
float prev_errorL = 0;
float integralL = 0;

//
//==================================================
// RPM
//==================================================
//

float rpmRight_raw = 0;
float rpmLeft_raw = 0;

float rpmRight_ema = 0;
float rpmLeft_ema = 0;

float alpha = 0.197;

//
//==================================================
// PWM
//==================================================
//

int pwmRight = 0;
int pwmLeft = 0;

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

  setupMotors();

  setupEncoders();

  tiempo_inicio = millis();

  Serial.println("DUAL PID READY");
}

//
//==================================================
// LOOP
//==================================================
//

void loop() {

  static unsigned long lastTime = 0;

  if (millis() - lastTime >= 100) {

    //--------------------------------
    // RPM
    //--------------------------------

    updateRPM();

    //--------------------------------
    // FILTER
    //--------------------------------

    applyEMAFilter();

    //--------------------------------
    // STARTUP
    //--------------------------------

    if (millis() - tiempo_inicio < 1000) {

      applyStartupPWM();
    }

    //--------------------------------
    // PID
    //--------------------------------

    else {

      updatePIDRight();
      updatePIDLeft();

      applyPWM();
    }

    //--------------------------------
    // SERIAL
    //--------------------------------

    printData();

    lastTime = millis();
  }
}

//
//==================================================
// SETUP MOTORS
//==================================================
//

void setupMotors() {

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  //--------------------------------
  // DIRECCIONES
  //--------------------------------

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  //--------------------------------
  // PWM RIGHT
  //--------------------------------

  ledcSetup(
      PWM_CHANNEL_RIGHT,
      PWM_FREQ,
      PWM_RESOLUTION);

  ledcAttachPin(
      PWMB,
      PWM_CHANNEL_RIGHT);

  //--------------------------------
  // PWM LEFT
  //--------------------------------

  ledcSetup(
      PWM_CHANNEL_LEFT,
      PWM_FREQ,
      PWM_RESOLUTION);

  ledcAttachPin(
      PWMA,
      PWM_CHANNEL_LEFT);
}

//
//==================================================
// SETUP ENCODERS
//==================================================
//

void setupEncoders() {

  pinMode(ENCODER_RIGHT_A, INPUT);
  pinMode(ENCODER_RIGHT_B, INPUT);

  pinMode(ENCODER_LEFT_A, INPUT);
  pinMode(ENCODER_LEFT_B, INPUT);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_RIGHT_A),
      handleRightEncoderA,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_RIGHT_B),
      handleRightEncoderB,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_LEFT_A),
      handleLeftEncoderA,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_LEFT_B),
      handleLeftEncoderB,
      CHANGE);
}

//
//==================================================
// UPDATE RPM
//==================================================
//

void updateRPM() {

  static long lastRight = 0;
  static long lastLeft = 0;

  long rightPos;
  long leftPos;

  noInterrupts();

  rightPos = encoderRightPos;
  leftPos = encoderLeftPos;

  interrupts();

  //--------------------------------
  // DELTAS
  //--------------------------------

  long deltaRight = rightPos - lastRight;
  long deltaLeft = leftPos - lastLeft;

  //--------------------------------
  // RPM
  //--------------------------------

  rpmRight_raw =
      (deltaRight / PPR) *
      (60000.0 / 100.0);

  rpmLeft_raw =
      (deltaLeft / PPR) *
      (60000.0 / 100.0);

  //--------------------------------
  // SIGNO LEFT
  //--------------------------------

  rpmLeft_raw = -rpmLeft_raw;

  //--------------------------------
  // UPDATE
  //--------------------------------

  lastRight = rightPos;
  lastLeft = leftPos;
}

//
//==================================================
// EMA FILTER
//==================================================
//

void applyEMAFilter() {

  rpmRight_ema =
      alpha * rpmRight_raw +
      (1.0 - alpha) * rpmRight_ema;

  rpmLeft_ema =
      alpha * rpmLeft_raw +
      (1.0 - alpha) * rpmLeft_ema;
}

//
//==================================================
// STARTUP PWM
//==================================================
//

void applyStartupPWM() {

  pwmRight =
      (setpointRight * 225.0) / 130.0;

  pwmLeft =
      (setpointLeft * 225.0) / 130.0;

  if (pwmRight > 225)
    pwmRight = 225;

  if (pwmLeft > 225)
    pwmLeft = 225;

  applyPWM();
}

//
//==================================================
// PID RIGHT
//==================================================
//

void updatePIDRight() {

  errorR =
      setpointRight - rpmRight_ema;

  integralR +=
      errorR * SAMPLE_TIME;

  float derivative =
      (errorR - prev_errorR) /
      SAMPLE_TIME;

  float output =
      kpR * errorR +
      kiR * integralR +
      kdR * derivative;

  pwmRight +=
      (int)output;

  if (pwmRight > 255)
    pwmRight = 255;

  if (pwmRight < 0)
    pwmRight = 0;

  prev_errorR = errorR;
}

//
//==================================================
// PID LEFT
//==================================================
//

void updatePIDLeft() {

  errorL =
      setpointLeft - rpmLeft_ema;

  integralL +=
      errorL * SAMPLE_TIME;

  float derivative =
      (errorL - prev_errorL) /
      SAMPLE_TIME;

  float output =
      kpL * errorL +
      kiL * integralL +
      kdL * derivative;

  pwmLeft +=
      (int)output;

  if (pwmLeft > 255)
    pwmLeft = 255;

  if (pwmLeft < 0)
    pwmLeft = 0;

  prev_errorL = errorL;
}

//
//==================================================
// APPLY PWM
//==================================================
//

void applyPWM() {

  ledcWrite(
      PWM_CHANNEL_RIGHT,
      pwmRight);

  ledcWrite(
      PWM_CHANNEL_LEFT,
      pwmLeft);
}

//
//==================================================
// SERIAL
//==================================================
//

void printData() {

  Serial.print("RIGHT RPM: ");
  Serial.print(rpmRight_ema);

  Serial.print(" | PWM: ");
  Serial.print(pwmRight);

  Serial.print(" || LEFT RPM: ");
  Serial.print(rpmLeft_ema);

  Serial.print(" | PWM: ");
  Serial.println(pwmLeft);
}

//
//==================================================
// ISR RIGHT
//==================================================
//

void handleRightEncoderA() {

  bool A = digitalRead(ENCODER_RIGHT_A);
  bool B = digitalRead(ENCODER_RIGHT_B);

  if (A == B)
    encoderRightPos++;
  else
    encoderRightPos--;
}

void handleRightEncoderB() {

  bool A = digitalRead(ENCODER_RIGHT_A);
  bool B = digitalRead(ENCODER_RIGHT_B);

  if (A != B)
    encoderRightPos++;
  else
    encoderRightPos--;
}

//
//==================================================
// ISR LEFT
//==================================================
//

void handleLeftEncoderA() {

  bool A = digitalRead(ENCODER_LEFT_A);
  bool B = digitalRead(ENCODER_LEFT_B);

  if (A == B)
    encoderLeftPos++;
  else
    encoderLeftPos--;
}

void handleLeftEncoderB() {

  bool A = digitalRead(ENCODER_LEFT_A);
  bool B = digitalRead(ENCODER_LEFT_B);

  if (A != B)
    encoderLeftPos++;
  else
    encoderLeftPos--;
}