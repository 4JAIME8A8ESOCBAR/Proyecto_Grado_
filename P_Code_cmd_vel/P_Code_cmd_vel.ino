#include <micro_ros_arduino.h>

#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

//
//==================================================
// MOTOR PINS
//==================================================
//

#define PWMR 26
#define R_IN1 27
#define R_IN2 14

#define PWML 32
#define L_IN1 33
#define L_IN2 25

#define STBY 13

//
//==================================================
// ENCODERS
//==================================================
//

#define ENCODER_R_A 34
#define ENCODER_R_B 35

#define ENCODER_L_A 39
#define ENCODER_L_B 36

//
//==================================================
// PWM
//==================================================
//

#define PWM_FREQ 15000
#define PWM_RES 8

#define CH_R 0
#define CH_L 1

#define MAX_PWM 255

//
//==================================================
// ROBOT PARAMS
//==================================================
//

const float PPR = 4230.0;

const float WHEEL_RADIUS = 0.03235;
const float WHEEL_BASE = 0.18;

//
//==================================================
// CONTROL LOOP
//==================================================
//

const float CONTROL_DT = 0.02;   // 20 ms
const float CONTROL_HZ = 50.0;

//
//==================================================
// MICRO ROS
//==================================================
//

rcl_subscription_t sub_cmdvel;
rcl_publisher_t pub_odom;

geometry_msgs__msg__Twist msg_cmdvel;
nav_msgs__msg__Odometry odom_msg;

rclc_executor_t executor;
rclc_support_t support;

rcl_allocator_t allocator;
rcl_node_t node;

//
//==================================================
// ERROR MACROS
//==================================================
//

#define RCCHECK(fn)                           \
  {                                           \
    rcl_ret_t temp_rc = fn;                   \
    if ((temp_rc != RCL_RET_OK)) {            \
      error_loop();                           \
    }                                         \
  }

#define RCSOFTCHECK(fn)                       \
  {                                           \
    rcl_ret_t temp_rc = fn;                   \
    if((temp_rc != RCL_RET_OK)){}             \
    (void)temp_rc;                            \
  }

//
//==================================================
// ENCODERS
//==================================================
//

volatile long encR = 0;
volatile long encL = 0;

long lastEncR = 0;
long lastEncL = 0;

//
//==================================================
// RPM
//==================================================
//

float rpmR = 0;
float rpmL = 0;

float rpmR_f = 0;
float rpmL_f = 0;

float alphaRPM = 0.25;

//
//==================================================
// PID
//==================================================
//

float setR = 0;
float setL = 0;

float kp = 2.52;
float ki = 2.08;
float kd = 1.30;

float eR = 0;
float eL = 0;

float eR_prev = 0;
float eL_prev = 0;

float iR = 0;
float iL = 0;

float pwmR = 0;
float pwmL = 0;

//
//==================================================
// ODOMETRY
//==================================================
//

float x = 0;
float y = 0;
float theta = 0;

float linearVel = 0;
float angularVel = 0;

//
//==================================================
// CMD TIMEOUT
//==================================================
//

unsigned long lastCmdVelTime = 0;

const unsigned long CMD_TIMEOUT = 500;

//
//==================================================
// ERROR LOOP
//==================================================
//

void error_loop() {

  while (1) {

    digitalWrite(STBY, !digitalRead(STBY));

    delay(100);
  }
}

//
//==================================================
// CMD_VEL CALLBACK
//==================================================
//

void cmdvel_callback(const void *msgin) {

  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;

  //--------------------------------
  // SAVE TIME
  //--------------------------------

  lastCmdVelTime = millis();

  //--------------------------------
  // CMD VEL
  //--------------------------------

  float v = msg->linear.x;
  float w = msg->angular.z;

  //--------------------------------
  // DIFFERENTIAL DRIVE
  //--------------------------------

  float vR =
      v + (WHEEL_BASE / 2.0) * w;

  float vL =
      v - (WHEEL_BASE / 2.0) * w;

  //--------------------------------
  // M/S -> RPM
  //--------------------------------

  setR =
      (vR * 60.0) /
      (2.0 * PI * WHEEL_RADIUS);

  setL =
      (vL * 60.0) /
      (2.0 * PI * WHEEL_RADIUS);
}

//
//==================================================
// SETUP
//==================================================
//

void setup() {

  //--------------------------------
  // SERIAL
  //--------------------------------

  Serial.begin(115200);

  delay(2000);

  //--------------------------------
  // MICRO ROS
  //--------------------------------

  set_microros_transports();

  //--------------------------------
  // STBY
  //--------------------------------

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  //--------------------------------
  // HARDWARE
  //--------------------------------

  setupMotors();

  setupPWM();

  setupEncoders();

  //--------------------------------
  // ROS
  //--------------------------------

  allocator =
      rcl_get_default_allocator();

  //--------------------------------
  // SUPPORT
  //--------------------------------

  RCCHECK(
      rclc_support_init(
          &support,
          0,
          NULL,
          &allocator));

  //--------------------------------
  // NODE
  //--------------------------------

  RCCHECK(
      rclc_node_init_default(
          &node,
          "esp32_robot",
          "",
          &support));

  //--------------------------------
  // CREATE SUBSCRIBER
  //--------------------------------

  RCCHECK(
      rclc_subscription_init_default(
          &sub_cmdvel,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(
              geometry_msgs,
              msg,
              Twist),
          "/cmd_vel"));

  //--------------------------------
  // ODOM PUBLISHER
  //--------------------------------

  RCCHECK(
      rclc_publisher_init_default(
          &pub_odom,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(
              nav_msgs,
              msg,
              Odometry),
          "/odom"));

  //--------------------------------
  // EXECUTOR
  //--------------------------------

  RCCHECK(
      rclc_executor_init(
          &executor,
          &support.context,
          1,
          &allocator));

  //--------------------------------
  // ADD SUB
  //--------------------------------

  RCCHECK(
      rclc_executor_add_subscription(
          &executor,
          &sub_cmdvel,
          &msg_cmdvel,
          &cmdvel_callback,
          ON_NEW_DATA));

  //--------------------------------
  // INIT
  //--------------------------------

  lastCmdVelTime = millis();

  //--------------------------------
  // READY
  //--------------------------------

  Serial.println("ROS READY");
}

//
//==================================================
// LOOP
//==================================================
//

void loop() {

  //--------------------------------
  // MICRO ROS
  //--------------------------------

  RCSOFTCHECK(
      rclc_executor_spin_some(
          &executor,
          RCL_MS_TO_NS(5)));

  //--------------------------------
  // CONTROL LOOP 50 Hz
  //--------------------------------

  static unsigned long lastControl = 0;

  if (millis() - lastControl >= 20) {

    //--------------------------------
    // SAFETY TIMEOUT
    //--------------------------------

    checkCmdTimeout();

    //--------------------------------
    // RPM
    //--------------------------------

    updateRPM();

    //--------------------------------
    // FILTER
    //--------------------------------

    filterRPM();

    //--------------------------------
    // PID
    //--------------------------------

    updatePID();

    //--------------------------------
    // PWM
    //--------------------------------

    applyPWM();

    //--------------------------------
    // ODOM
    //--------------------------------

    updateOdometry();

    //--------------------------------
    // PUBLISH ODOM
    //--------------------------------

    publishOdometry();

    //--------------------------------
    // DEBUG
    //--------------------------------

    printDebug();

    //--------------------------------
    // TIME
    //--------------------------------

    lastControl = millis();
  }
}

//
//==================================================
// CMD TIMEOUT
//==================================================
//

void checkCmdTimeout() {

  if (millis() - lastCmdVelTime > CMD_TIMEOUT) {

    setR = 0;
    setL = 0;
  }
}

//
//==================================================
// SETUP MOTORS
//==================================================
//

void setupMotors() {

  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);

  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);

  //--------------------------------
  // RIGHT
  //--------------------------------

  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, HIGH);

  //--------------------------------
  // LEFT
  //--------------------------------

  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
}

//
//==================================================
// PWM
//==================================================
//

void setupPWM() {

  //--------------------------------
  // RIGHT
  //--------------------------------

  ledcSetup(
      CH_R,
      PWM_FREQ,
      PWM_RES);

  ledcAttachPin(
      PWMR,
      CH_R);

  //--------------------------------
  // LEFT
  //--------------------------------

  ledcSetup(
      CH_L,
      PWM_FREQ,
      PWM_RES);

  ledcAttachPin(
      PWML,
      CH_L);
}

//
//==================================================
// ENCODERS
//==================================================
//

void setupEncoders() {

  pinMode(ENCODER_R_A, INPUT);
  pinMode(ENCODER_R_B, INPUT);

  pinMode(ENCODER_L_A, INPUT);
  pinMode(ENCODER_L_B, INPUT);

  //--------------------------------
  // RIGHT
  //--------------------------------

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_R_A),
      encRA,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_R_B),
      encRB,
      CHANGE);

  //--------------------------------
  // LEFT
  //--------------------------------

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_L_A),
      encLA,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_L_B),
      encLB,
      CHANGE);
}

//
//==================================================
// UPDATE RPM
//==================================================
//

void updateRPM() {

  long r;
  long l;

  noInterrupts();

  r = encR;
  l = encL;

  interrupts();

  //--------------------------------
  // DELTAS
  //--------------------------------

  long deltaR =
      r - lastEncR;

  long deltaL =
      l - lastEncL;

  //--------------------------------
  // RPM
  //--------------------------------

  rpmR =
      (deltaR / PPR) *
      (60.0 * CONTROL_HZ);

  rpmL =
      (deltaL / PPR) *
      (60.0 * CONTROL_HZ);

  //--------------------------------
  // LEFT SIGN
  //--------------------------------

  rpmL = -rpmL;

  //--------------------------------
  // SAVE
  //--------------------------------

  lastEncR = r;
  lastEncL = l;
}

//
//==================================================
// FILTER RPM
//==================================================
//

void filterRPM() {

  rpmR_f =
      alphaRPM * rpmR +
      (1.0 - alphaRPM) * rpmR_f;

  //--------------------------------

  rpmL_f =
      alphaRPM * rpmL +
      (1.0 - alphaRPM) * rpmL_f;
}

//
//==================================================
// PID
//==================================================
//

void updatePID() {

  //--------------------------------
  // RIGHT
  //--------------------------------

  eR =
      setR - rpmR_f;

  iR +=
      eR * CONTROL_DT;

  //--------------------------------
  // ANTI WINDUP
  //--------------------------------

  if (iR > 100) iR = 100;
  if (iR < -100) iR = -100;

  //--------------------------------

  float derR =
      (eR - eR_prev) / CONTROL_DT;

  //--------------------------------

  float outR =
      kp * eR +
      ki * iR +
      kd * derR;

  //--------------------------------

  pwmR += outR;

  //--------------------------------

  if (pwmR > MAX_PWM)
    pwmR = MAX_PWM;

  if (pwmR < 0)
    pwmR = 0;

  //--------------------------------

  eR_prev = eR;

  //--------------------------------
  // LEFT
  //--------------------------------

  eL =
      setL - rpmL_f;

  iL +=
      eL * CONTROL_DT;

  //--------------------------------
  // ANTI WINDUP
  //--------------------------------

  if (iL > 100) iL = 100;
  if (iL < -100) iL = -100;

  //--------------------------------

  float derL =
      (eL - eL_prev) / CONTROL_DT;

  //--------------------------------

  float outL =
      kp * eL +
      ki * iL +
      kd * derL;

  //--------------------------------

  pwmL += outL;

  //--------------------------------

  if (pwmL > MAX_PWM)
    pwmL = MAX_PWM;

  if (pwmL < 0)
    pwmL = 0;

  //--------------------------------

  eL_prev = eL;
}

//
//==================================================
// APPLY PWM
//==================================================
//

void applyPWM() {

  ledcWrite(CH_R, (int)pwmR);

  ledcWrite(CH_L, (int)pwmL);
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

  float omegaR =
      (2.0 * PI * rpmR_f) / 60.0;

  float omegaL =
      (2.0 * PI * rpmL_f) / 60.0;

  //--------------------------------
  // WHEEL SPEED
  //--------------------------------

  float vR =
      WHEEL_RADIUS * omegaR;

  float vL =
      WHEEL_RADIUS * omegaL;

  //--------------------------------
  // ROBOT SPEED
  //--------------------------------

  linearVel =
      (vR + vL) / 2.0;

  angularVel =
      (vR - vL) / WHEEL_BASE;

  //--------------------------------
  // POSE
  //--------------------------------

  theta +=
      angularVel * CONTROL_DT;

  //--------------------------------

  x +=
      linearVel *
      cos(theta) *
      CONTROL_DT;

  //--------------------------------

  y +=
      linearVel *
      sin(theta) *
      CONTROL_DT;
}

//
//==================================================
// PUBLISH ODOM
//==================================================
//

void publishOdometry() {

  //--------------------------------
  // POSITION
  //--------------------------------

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;

  //--------------------------------
  // SIMPLE YAW -> QUAT
  //--------------------------------

  odom_msg.pose.pose.orientation.z =
      sin(theta / 2.0);

  odom_msg.pose.pose.orientation.w =
      cos(theta / 2.0);

  //--------------------------------
  // VELOCITIES
  //--------------------------------

  odom_msg.twist.twist.linear.x =
      linearVel;

  odom_msg.twist.twist.angular.z =
      angularVel;

  //--------------------------------
  // PUBLISH
  //--------------------------------

  RCSOFTCHECK(
      rcl_publish(
          &pub_odom,
          &odom_msg,
          NULL));
}

//
//==================================================
// DEBUG
//==================================================
//

void printDebug() {

  Serial.print("SET_R:");
  Serial.print(setR);

  Serial.print(" RPM_R:");
  Serial.print(rpmR_f);

  Serial.print(" PWM_R:");
  Serial.print(pwmR);

  Serial.print(" | ");

  Serial.print("SET_L:");
  Serial.print(setL);

  Serial.print(" RPM_L:");
  Serial.print(rpmL_f);

  Serial.print(" PWM_L:");
  Serial.print(pwmL);

  Serial.print(" | X:");
  Serial.print(x);

  Serial.print(" Y:");
  Serial.print(y);

  Serial.print(" TH:");
  Serial.println(theta);
}

//
//==================================================
// ISR RIGHT
//==================================================
//

void encRA() {

  bool A =
      digitalRead(ENCODER_R_A);

  bool B =
      digitalRead(ENCODER_R_B);

  if (A == B)
    encR++;
  else
    encR--;
}

void encRB() {

  bool A =
      digitalRead(ENCODER_R_A);

  bool B =
      digitalRead(ENCODER_R_B);

  if (A != B)
    encR++;
  else
    encR--;
}

//
//==================================================
// ISR LEFT
//==================================================
//

void encLA() {

  bool A =
      digitalRead(ENCODER_L_A);

  bool B =
      digitalRead(ENCODER_L_B);

  if (A == B)
    encL++;
  else
    encL--;
}

void encLB() {

  bool A =
      digitalRead(ENCODER_L_A);

  bool B =
      digitalRead(ENCODER_L_B);

  if (A != B)
    encL++;
  else
    encL--;
}