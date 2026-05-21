//
//==================================================
// ROBOT DIFERENCIAL + PID + ODOMETRIA
//==================================================
// ESP32 + TB6612FNG + ENCODERS
//==================================================
//

#include <math.h>

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
// PWM CONFIG
//==================================================
//

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

#define PWM_CHANNEL_RIGHT 0
#define PWM_CHANNEL_LEFT 1

//
//==================================================
// ROBOT PARAMS
//==================================================
//

const float PPR = 4230.0;

const float WHEEL_RADIUS = 0.03235;
const float WHEEL_BASE   = 0.18;

const float DT = 0.1;

//
//==================================================
// ENCODERS
//==================================================
//

volatile long encoderRightPos = 0;
volatile long encoderLeftPos  = 0;

//
//==================================================
// RPM
//==================================================
//

float rpmRight_raw = 0;
float rpmLeft_raw  = 0;

float rpmRight_ema = 0;
float rpmLeft_ema  = 0;

float alpha = 0.197;

//
//==================================================
// PID RIGHT
//==================================================
//

float setpointRight = 100.0;

float kpRight = 2.52522;
float kiRight = 2.0829;
float kdRight = 1.305;

float errorRight = 0;
float prevErrorRight = 0;
float integralRight = 0;

//
//==================================================
// PID LEFT
//==================================================
//

float setpointLeft = 100.0;

float kpLeft = 2.52522;
float kiLeft = 2.0829;
float kdLeft = 1.305;

float errorLeft = 0;
float prevErrorLeft = 0;
float integralLeft = 0;

//
//==================================================
// PWM
//==================================================
//

int pwmRight = 0;
int pwmLeft  = 0;

//
//==================================================
// ODOM
//==================================================
//

float x = 0.0;
float y = 0.0;
float theta = 0.0;

float linearVelocity  = 0.0;
float angularVelocity = 0.0;

//
//==================================================
// SETUP
//==================================================
//

void setup() {

  Serial.begin(115200);

  setupMotors();

  setupPWM();

  setupEncoders();

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

    //--------------------------------
    // RPM
    //--------------------------------

    updateRPM();

    //--------------------------------
    // FILTRO
    //--------------------------------

    applyEMAFilter();

    //--------------------------------
    // PID
    //--------------------------------

    updatePIDRight();
    updatePIDLeft();

    //--------------------------------
    // PWM
    //--------------------------------

    applyPWM();

    //--------------------------------
    // ODOM
    //--------------------------------

    updateOdometry();

    //--------------------------------
    // SERIAL
    //--------------------------------

    printOdometry();

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
  // RIGHT
  //--------------------------------

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  //--------------------------------
  // LEFT
  //--------------------------------

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
}

//
//==================================================
// PWM
//==================================================
//

void setupPWM() {

  ledcSetup(
      PWM_CHANNEL_RIGHT,
      PWM_FREQ,
      PWM_RESOLUTION);

  ledcAttachPin(
      PWMB,
      PWM_CHANNEL_RIGHT);

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
// ENCODERS
//==================================================
//

void setupEncoders() {

  pinMode(ENCODER_RIGHT_A, INPUT);
  pinMode(ENCODER_RIGHT_B, INPUT);

  pinMode(ENCODER_LEFT_A, INPUT);
  pinMode(ENCODER_LEFT_B, INPUT);

  //--------------------------------
  // RIGHT
  //--------------------------------

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_RIGHT_A),
      handleRightEncoderA,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_RIGHT_B),
      handleRightEncoderB,
      CHANGE);

  //--------------------------------
  // LEFT
  //--------------------------------

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
// RPM
//==================================================
//

void updateRPM() {

  static long lastRight = 0;
  static long lastLeft  = 0;

  long rightPos;
  long leftPos;

  noInterrupts();

  rightPos = encoderRightPos;
  leftPos  = encoderLeftPos;

  interrupts();

  //--------------------------------
  // DELTAS
  //--------------------------------

  long deltaRight = rightPos - lastRight;
  long deltaLeft  = leftPos - lastLeft;

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
  // SIGNO
  //--------------------------------

  rpmLeft_raw = -rpmLeft_raw;

  //--------------------------------
  // SAVE
  //--------------------------------

  lastRight = rightPos;
  lastLeft  = leftPos;
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

  //--------------------------------

  rpmLeft_ema =
      alpha * rpmLeft_raw +
      (1.0 - alpha) * rpmLeft_ema;
}

//
//==================================================
// PID RIGHT
//==================================================
//

void updatePIDRight() {

  errorRight =
      setpointRight - rpmRight_ema;

  //--------------------------------

  integralRight +=
      errorRight * DT;

  //--------------------------------

  float derivative =
      (errorRight - prevErrorRight) / DT;

  //--------------------------------

  float output =
      kpRight * errorRight +
      kiRight * integralRight +
      kdRight * derivative;

  //--------------------------------

  pwmRight += (int)output;

  //--------------------------------

  if (pwmRight > 225)
    pwmRight = 225;

  if (pwmRight < 0)
    pwmRight = 0;

  //--------------------------------

  prevErrorRight = errorRight;
}

//
//==================================================
// PID LEFT
//==================================================
//

void updatePIDLeft() {

  errorLeft =
      setpointLeft - rpmLeft_ema;

  //--------------------------------

  integralLeft +=
      errorLeft * DT;

  //--------------------------------

  float derivative =
      (errorLeft - prevErrorLeft) / DT;

  //--------------------------------

  float output =
      kpLeft * errorLeft +
      kiLeft * integralLeft +
      kdLeft * derivative;

  //--------------------------------

  pwmLeft += (int)output;

  //--------------------------------

  if (pwmLeft > 225)
    pwmLeft = 225;

  if (pwmLeft < 0)
    pwmLeft = 0;

  //--------------------------------

  prevErrorLeft = errorLeft;
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

  //--------------------------------

  ledcWrite(
      PWM_CHANNEL_LEFT,
      pwmLeft);
}

//
//==================================================
// ODOMETRY
//==================================================
//

void updateOdometry() {

  //--------------------------------
  // RPM -> RAD/S
  //--------------------------------

  float omegaRight =
      (2.0 * PI * rpmRight_ema) / 60.0;

  float omegaLeft =
      (2.0 * PI * rpmLeft_ema) / 60.0;

  //--------------------------------
  // VELOCIDADES RUEDAS
  //--------------------------------

  float vRight =
      WHEEL_RADIUS * omegaRight;

  float vLeft =
      WHEEL_RADIUS * omegaLeft;

  //--------------------------------
  // ROBOT
  //--------------------------------

  linearVelocity =
      (vRight + vLeft) / 2.0;

  angularVelocity =
      (vRight - vLeft) / WHEEL_BASE;

  //--------------------------------
  // POSE
  //--------------------------------

  theta +=
      angularVelocity * DT;

  //--------------------------------

  x +=
      linearVelocity *
      cos(theta) *
      DT;

  //--------------------------------

  y +=
      linearVelocity *
      sin(theta) *
      DT;
}

//
//==================================================
// SERIAL
//==================================================
//

void printOdometry() {

  Serial.print(x);
  Serial.print(" ");

  Serial.print(y);
  Serial.print(" ");

  Serial.print(theta);
  Serial.print(" ");

  Serial.print(rpmRight_ema);
  Serial.print(" ");

  Serial.println(rpmLeft_ema);
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