#include <Arduino.h>

//==================================================
// TB6612FNG PINS
//==================================================

// RIGHT MOTOR
#define PWMB 26
#define BIN1 27
#define BIN2 14

// LEFT MOTOR
#define PWMA 32
#define AIN1 33
#define AIN2 25

// STANDBY
#define STBY 13

//==================================================
// ENCODERS
//==================================================

// RIGHT
#define ENC_R_A 39
#define ENC_R_B 36

// LEFT
#define ENC_L_A 34
#define ENC_L_B 35

//==================================================
// PWM CONFIG ESP32
//==================================================

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8

#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1

//==================================================
// ENCODER VARIABLES
//==================================================

volatile long encoderRight = 0;
volatile long encoderLeft = 0;

//==================================================
// PID VARIABLES
//==================================================

float setpointLeft = 100;
float setpointRight = 100;

// RIGHT PID
float kpR = 3.1;
float kiR = 2.0;
float kdR = 1.4;

// LEFT PID
float kpL = 3.1;
float kiL = 2.0;
float kdL = 1.4;

float integralR = 0;
float prevErrorR = 0;

float integralL = 0;
float prevErrorL = 0;

//==================================================
// PWM OUTPUT
//==================================================

int pwmRight = 0;
int pwmLeft = 0;

//==================================================
// FILTER EMA
//==================================================

float alpha = 0.2;

float rpmRightEMA = 0;
float rpmLeftEMA = 0;

//==================================================
// ENCODER PPR
//==================================================

// AJUSTAR SEGUN TU MOTOR
float pulsesPerRevolution = 3300.0;

//==================================================
// TIME
//==================================================

unsigned long lastControl = 0;

//==================================================
// SETUP
//==================================================

void setup() {

  Serial.begin(115200);

  //--------------------------------
  // STBY
  //--------------------------------

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  //--------------------------------
  // MOTOR PINS
  //--------------------------------

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  //--------------------------------
  // PWM ESP32
  //--------------------------------

  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA, PWM_CHANNEL_LEFT);

  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMB, PWM_CHANNEL_RIGHT);

  //--------------------------------
  // ENCODERS
  //--------------------------------

  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);

  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);

  attachInterrupt(
      digitalPinToInterrupt(ENC_R_A),
      handleRightA,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(ENC_R_B),
      handleRightB,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(ENC_L_A),
      handleLeftA,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(ENC_L_B),
      handleLeftB,
      CHANGE);

  //--------------------------------
  // DIRECCION ADELANTE
  //--------------------------------

  forwardLeft();
  forwardRight();

  Serial.println("ROBOT READY");
}

//==================================================
// LOOP
//==================================================

void loop() {

  if (millis() - lastControl >= 50) {

    controlMotors();

    lastControl = millis();
  }
}

//==================================================
// CONTROL PID
//==================================================

void controlMotors() {

  static long lastRight = 0;
  static long lastLeft = 0;

  //--------------------------------
  // READ ENCODERS
  //--------------------------------

  long currentRight;
  long currentLeft;

  noInterrupts();

  currentRight = encoderRight;
  currentLeft = encoderLeft;

  interrupts();

  //--------------------------------
  // DELTA
  //--------------------------------

  long deltaRight = currentRight - lastRight;
  long deltaLeft = currentLeft - lastLeft;

  //--------------------------------
  // RPM
  //--------------------------------

  float rpmRight =
      (deltaRight / pulsesPerRevolution)
      * 1200.0;

  float rpmLeft =
      (deltaLeft / pulsesPerRevolution)
      * 1200.0;

  //--------------------------------
  // EMA FILTER
  //--------------------------------

  rpmRightEMA =
      alpha * rpmRight +
      (1 - alpha) * rpmRightEMA;

  rpmLeftEMA =
      alpha * rpmLeft +
      (1 - alpha) * rpmLeftEMA;

  //--------------------------------
  // PID RIGHT
  //--------------------------------

  float errorR =
      setpointRight - rpmRightEMA;

  integralR += errorR * 0.05;

  float derivativeR =
      (errorR - prevErrorR) / 0.05;

  float outputR =
      kpR * errorR +
      kiR * integralR +
      kdR * derivativeR;

  prevErrorR = errorR;

  pwmRight += outputR;

  //--------------------------------
  // PID LEFT
  //--------------------------------

  float errorL =
      setpointLeft - rpmLeftEMA;

  integralL += errorL * 0.05;

  float derivativeL =
      (errorL - prevErrorL) / 0.05;

  float outputL =
      kpL * errorL +
      kiL * integralL +
      kdL * derivativeL;

  prevErrorL = errorL;

  pwmLeft += outputL;

  //--------------------------------
  // LIMIT PWM
  //--------------------------------

  pwmRight = constrain(pwmRight, 0, 255);
  pwmLeft = constrain(pwmLeft, 0, 255);

  //--------------------------------
  // APPLY PWM
  //--------------------------------

  ledcWrite(PWM_CHANNEL_RIGHT, pwmRight);
  ledcWrite(PWM_CHANNEL_LEFT, pwmLeft);

  //--------------------------------
  // DEBUG
  //--------------------------------

  Serial.print("RPM_R:");
  Serial.print(rpmRightEMA);

  Serial.print(",");

  Serial.print("PWM_R:");
  Serial.print(pwmRight);

  Serial.print(",");

  Serial.print("RPM_L:");
  Serial.print(rpmLeftEMA);

  Serial.print(",");

  Serial.print("PWM_L:");
  Serial.println(pwmLeft);

  //--------------------------------
  // UPDATE
  //--------------------------------

  lastRight = currentRight;
  lastLeft = currentLeft;
}

//==================================================
// DIRECTIONS
//==================================================

void forwardRight() {

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void forwardLeft() {

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
}

//==================================================
// ENCODER INTERRUPTS
//==================================================

void handleRightA() {

  bool A = digitalRead(ENC_R_A);
  bool B = digitalRead(ENC_R_B);

  encoderRight += (A == B) ? 1 : -1;
}

void handleRightB() {

  bool A = digitalRead(ENC_R_A);
  bool B = digitalRead(ENC_R_B);

  encoderRight += (A != B) ? 1 : -1;
}

void handleLeftA() {

  bool A = digitalRead(ENC_L_A);
  bool B = digitalRead(ENC_L_B);

  encoderLeft += (A == B) ? 1 : -1;
}

void handleLeftB() {

  bool A = digitalRead(ENC_L_A);
  bool B = digitalRead(ENC_L_B);

  encoderLeft += (A != B) ? 1 : -1;
}