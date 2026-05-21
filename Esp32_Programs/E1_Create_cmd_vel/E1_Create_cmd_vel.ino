//
//==================================================
// ETAPA 1
// ROS2 + micro-ROS + /cmd_vel
// PID + Encoders + Teleop
// BASE PARA SLAM
//==================================================
//

#include <micro_ros_arduino.h>

#include <math.h>
#include <Wire.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

//
//==================================================
// MOTOR RIGHT
//==================================================
//

#define PWMB 26
#define BIN1 27
#define BIN2 14

//
//==================================================
// MOTOR LEFT
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
// ROBOT PARAMS
//==================================================
//

const float PPR = 4230.0;

const float WHEEL_RADIUS = 0.03235;
const float WHEEL_BASE   = 0.18;

//
//==================================================
// CONTROL FREQUENCY
//==================================================
//

const float CONTROL_PERIOD = 0.02; // 20ms = 50Hz

//
//==================================================
// LIMITS
//==================================================
//

const float MAX_LINEAR  = 0.35;
const float MAX_ANGULAR = 2.5;

const int MAX_PWM = 220;
const int MIN_PWM = 35;

//
//==================================================
// CMD_VEL WATCHDOG
//==================================================
//

const unsigned long CMD_TIMEOUT = 500;

unsigned long lastCmdVelTime = 0;

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

float alphaRPM = 0.2;

//
//==================================================
// TARGETS
//==================================================
//

float targetLinear  = 0.0;
float targetAngular = 0.0;

float targetRPMRight = 0.0;
float targetRPMLeft  = 0.0;

//
//==================================================
// PID RIGHT
//==================================================
//

float kpRight = 2.5;
float kiRight = 2.0;
float kdRight = 1.2;

float errorRight = 0;
float prevErrorRight = 0;
float integralRight = 0;

//
//==================================================
// PID LEFT
//==================================================
//

float kpLeft = 2.5;
float kiLeft = 2.0;
float kdLeft = 1.2;

float errorLeft = 0;
float prevErrorLeft = 0;
float integralLeft = 0;

//
//==================================================
// PWM OUTPUT
//==================================================
//

int pwmRight = 0;
int pwmLeft  = 0;

//
//==================================================
// micro-ROS
//==================================================
//

rcl_allocator_t allocator;
rclc_support_t support;

rcl_node_t node;

rcl_subscription_t subscriber;

rclc_executor_t executor;

geometry_msgs__msg__Twist cmd_msg;

//
//==================================================
// CALLBACK CMD_VEL
//==================================================
//

void cmdVelCallback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg =
    (const geometry_msgs__msg__Twist *)msgin;

  //--------------------------------
  // SAVE TIME
  //--------------------------------

  lastCmdVelTime = millis();

  //--------------------------------
  // LIMITS
  //--------------------------------

  targetLinear =
    constrain(
      msg->linear.x,
      -MAX_LINEAR,
      MAX_LINEAR);

  //--------------------------------

  targetAngular =
    constrain(
      msg->angular.z,
      -MAX_ANGULAR,
      MAX_ANGULAR);

  //--------------------------------
  // DEADBAND
  //--------------------------------

  if (fabs(targetLinear) < 0.01)
    targetLinear = 0.0;

  if (fabs(targetAngular) < 0.01)
    targetAngular = 0.0;

  //--------------------------------
  // DIFFERENTIAL DRIVE
  //--------------------------------

  float vRight =
    targetLinear +
    (WHEEL_BASE / 2.0) * targetAngular;

  //--------------------------------

  float vLeft =
    targetLinear -
    (WHEEL_BASE / 2.0) * targetAngular;

  //--------------------------------
  // LINEAR TO RPM
  //--------------------------------

  targetRPMRight =
    (vRight * 60.0) /
    (2.0 * PI * WHEEL_RADIUS);

  //--------------------------------

  targetRPMLeft =
    (vLeft * 60.0) /
    (2.0 * PI * WHEEL_RADIUS);
}

//
//==================================================
// SETUP
//==================================================
//

void setup()
{
  Serial.begin(115200);

  //--------------------------------
  // micro-ROS
  //--------------------------------

  set_microros_transports();

  delay(2000);

  //--------------------------------
  // MOTORS
  //--------------------------------

  setupMotors();

  setupPWM();

  setupEncoders();

  //--------------------------------
  // micro-ROS init
  //--------------------------------

  allocator = rcl_get_default_allocator();

  rclc_support_init(
    &support,
    0,
    NULL,
    &allocator);

  //--------------------------------

  rclc_node_init_default(
    &node,
    "esp32_robot",
    "",
    &support);

  //--------------------------------

  rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      geometry_msgs,
      msg,
      Twist),
    "/cmd_vel");

  //--------------------------------

  rclc_executor_init(
    &executor,
    &support.context,
    1,
    &allocator);

  //--------------------------------

  rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &cmd_msg,
    &cmdVelCallback,
    ON_NEW_DATA);

  //--------------------------------

  lastCmdVelTime = millis();

  Serial.println("ROBOT READY");
}

//
//==================================================
// LOOP
//==================================================
//

void loop()
{
  //--------------------------------
  // micro-ROS
  //--------------------------------

  rclc_executor_spin_some(
    &executor,
    RCL_MS_TO_NS(5));

  //--------------------------------
  // CONTROL LOOP
  //--------------------------------

  static unsigned long lastControl = 0;

  unsigned long now = millis();

  if ((now - lastControl) >= 20)
  {
    float dt =
      (now - lastControl) / 1000.0;

    lastControl = now;

    //--------------------------------
    // WATCHDOG
    //--------------------------------

    if ((millis() - lastCmdVelTime)
        > CMD_TIMEOUT)
    {
      targetRPMRight = 0;
      targetRPMLeft  = 0;
    }

    //--------------------------------
    // RPM
    //--------------------------------

    updateRPM(dt);

    applyRPMFilter();

    //--------------------------------
    // PID
    //--------------------------------

    pwmRight =
      computePIDRight(dt);

    pwmLeft =
      computePIDLeft(dt);

    //--------------------------------
    // APPLY
    //--------------------------------

    setMotorRight(pwmRight);

    setMotorLeft(pwmLeft);

    //--------------------------------
    // DEBUG
    //--------------------------------

    static unsigned long lastPrint = 0;

    if (millis() - lastPrint > 500)
    {
      
      Serial.print("TR:");
      '''
      Serial.print(targetRPMRight);

      Serial.print(" RR:");
      Serial.print(rpmRight_ema);

      Serial.print(" TL:");
      Serial.print(targetRPMLeft);

      Serial.print(" RL:");
      Serial.println(rpmLeft_ema);
      '''
      lastPrint = millis();
    }
  }
}

//
//==================================================
// MOTOR SETUP
//==================================================
//

void setupMotors()
{
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
}

//
//==================================================
// PWM
//==================================================
//

void setupPWM()
{
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

void setupEncoders()
{
  pinMode(ENCODER_RIGHT_A, INPUT);
  pinMode(ENCODER_RIGHT_B, INPUT);

  pinMode(ENCODER_LEFT_A, INPUT);
  pinMode(ENCODER_LEFT_B, INPUT);

  //--------------------------------

  attachInterrupt(
    digitalPinToInterrupt(
      ENCODER_RIGHT_A),
    handleRightEncoderA,
    CHANGE);

  attachInterrupt(
    digitalPinToInterrupt(
      ENCODER_RIGHT_B),
    handleRightEncoderB,
    CHANGE);

  //--------------------------------

  attachInterrupt(
    digitalPinToInterrupt(
      ENCODER_LEFT_A),
    handleLeftEncoderA,
    CHANGE);

  attachInterrupt(
    digitalPinToInterrupt(
      ENCODER_LEFT_B),
    handleLeftEncoderB,
    CHANGE);
}

//
//==================================================
// UPDATE RPM
//==================================================
//

void updateRPM(float dt)
{
  static long lastRight = 0;
  static long lastLeft  = 0;

  long rightPos;
  long leftPos;

  noInterrupts();

  rightPos = encoderRightPos;
  leftPos  = encoderLeftPos;

  interrupts();

  //--------------------------------

  long deltaRight =
    rightPos - lastRight;

  long deltaLeft =
    leftPos - lastLeft;

  //--------------------------------

  rpmRight_raw =
    (deltaRight / PPR) *
    (60.0 / dt);

  //--------------------------------

  rpmLeft_raw =
    (deltaLeft / PPR) *
    (60.0 / dt);

  //--------------------------------

  lastRight = rightPos;
  lastLeft  = leftPos;
}

//
//==================================================
// FILTER RPM
//==================================================
//

void applyRPMFilter()
{
  rpmRight_ema =
    alphaRPM * rpmRight_raw +
    (1.0 - alphaRPM) * rpmRight_ema;

  //--------------------------------

  rpmLeft_ema =
    alphaRPM * rpmLeft_raw +
    (1.0 - alphaRPM) * rpmLeft_ema;
}

//
//==================================================
// PID RIGHT
//==================================================
//

int computePIDRight(float dt)
{
  errorRight =
    targetRPMRight -
    rpmRight_ema;

  //--------------------------------

  integralRight +=
    errorRight * dt;

  //--------------------------------
  // ANTI WINDUP
  //--------------------------------

  integralRight =
    constrain(
      integralRight,
      -100,
      100);

  //--------------------------------

  float derivative =
    (errorRight -
     prevErrorRight) / dt;

  //--------------------------------

  float output =
    kpRight * errorRight +
    kiRight * integralRight +
    kdRight * derivative;

  //--------------------------------

  prevErrorRight =
    errorRight;

  //--------------------------------

  return constrain(
    (int)output,
    -MAX_PWM,
    MAX_PWM);
}

//
//==================================================
// PID LEFT
//==================================================
//

int computePIDLeft(float dt)
{
  errorLeft =
    targetRPMLeft -
    rpmLeft_ema;

  //--------------------------------

  integralLeft +=
    errorLeft * dt;

  //--------------------------------

  integralLeft =
    constrain(
      integralLeft,
      -100,
      100);

  //--------------------------------

  float derivative =
    (errorLeft -
     prevErrorLeft) / dt;

  //--------------------------------

  float output =
    kpLeft * errorLeft +
    kiLeft * integralLeft +
    kdLeft * derivative;

  //--------------------------------

  prevErrorLeft =
    errorLeft;

  //--------------------------------

  return constrain(
    (int)output,
    -MAX_PWM,
    MAX_PWM);
}

//
//==================================================
// RIGHT MOTOR
//==================================================
//

void setMotorRight(int pwm)
{
  bool forward = pwm >= 0;

  int pwmAbs = abs(pwm);

  //--------------------------------
  // DEADZONE
  //--------------------------------

  if (pwmAbs > 0 &&
      pwmAbs < MIN_PWM)
  {
    pwmAbs = MIN_PWM;
  }

  //--------------------------------

  pwmAbs =
    constrain(
      pwmAbs,
      0,
      MAX_PWM);

  //--------------------------------

  if (forward)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }

  //--------------------------------

  ledcWrite(
    PWM_CHANNEL_RIGHT,
    pwmAbs);
}

//
//==================================================
// LEFT MOTOR
//==================================================
//

void setMotorLeft(int pwm)
{
  bool forward = pwm >= 0;

  int pwmAbs = abs(pwm);

  //--------------------------------

  if (pwmAbs > 0 &&
      pwmAbs < MIN_PWM)
  {
    pwmAbs = MIN_PWM;
  }

  //--------------------------------

  pwmAbs =
    constrain(
      pwmAbs,
      0,
      MAX_PWM);

  //--------------------------------

  if (forward)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

  //--------------------------------

  ledcWrite(
    PWM_CHANNEL_LEFT,
    pwmAbs);
}

//
//==================================================
// ISR RIGHT
//==================================================
//

void handleRightEncoderA()
{
  bool A =
    digitalRead(
      ENCODER_RIGHT_A);

  bool B =
    digitalRead(
      ENCODER_RIGHT_B);

  if (A == B)
    encoderRightPos++;
  else
    encoderRightPos--;
}

void handleRightEncoderB()
{
  bool A =
    digitalRead(
      ENCODER_RIGHT_A);

  bool B =
    digitalRead(
      ENCODER_RIGHT_B);

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

void handleLeftEncoderA()
{
  bool A =
    digitalRead(
      ENCODER_LEFT_A);

  bool B =
    digitalRead(
      ENCODER_LEFT_B);

  if (A == B)
    encoderLeftPos++;
  else
    encoderLeftPos--;
}

void handleLeftEncoderB()
{
  bool A =
    digitalRead(
      ENCODER_LEFT_A);

  bool B =
    digitalRead(
      ENCODER_LEFT_B);

  if (A != B)
    encoderLeftPos++;
  else
    encoderLeftPos--;
}