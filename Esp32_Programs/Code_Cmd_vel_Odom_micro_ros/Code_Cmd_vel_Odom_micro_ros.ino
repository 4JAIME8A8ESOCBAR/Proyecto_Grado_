#include <micro_ros_arduino.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <math.h>

// ======================================================
// MOTORS
// ======================================================

#define PWMB 26
#define BIN1 27
#define BIN2 14

#define PWMA 32
#define AIN1 33
#define AIN2 25

#define STBY 13

// ======================================================
// ENCODERS
// ======================================================

#define ENC_R_A 34
#define ENC_R_B 35

#define ENC_L_A 39
#define ENC_L_B 36

volatile long encR = 0;
volatile long encL = 0;

// ======================================================
// PWM
// ======================================================

#define PWM_FREQ 5000
#define PWM_RES 8

#define CH_R 0
#define CH_L 1

// ======================================================
// ROBOT PARAMETERS
// ======================================================

const float WHEEL_RADIUS = 0.03235;
const float WHEEL_BASE   = 0.18;
const float PPR          = 4230.0;

// ======================================================
// MOTOR INVERSION
// ======================================================

#define INVERT_RIGHT false
#define INVERT_LEFT  true

// ======================================================
// CONTROL
// ======================================================

float targetRPM_R = 0.0;
float targetRPM_L = 0.0;

// TARGETS SUAVIZADOS
float smoothTargetR = 0.0;
float smoothTargetL = 0.0;

float rpmR = 0.0;
float rpmL = 0.0;

float rpmR_f = 0.0;
float rpmL_f = 0.0;

float alpha = 0.25;

// ======================================================
// PID
// ======================================================

float kp = 1.8;//2.52522;
float ki = 0.8;//2.0829;
float kd = 0.15;//1.305;

float eR = 0;
float eL = 0;

float eR_prev = 0;
float eL_prev = 0;

float iR = 0;
float iL = 0;

int pwmR = 0;
int pwmL = 0;

// ======================================================
// PWM LIMITS
// ======================================================

const int MAX_PWM = 220;
const int MIN_PWM = 35;

// ======================================================
// TARGET RAMP
// ======================================================

// RPM máximas que pueden cambiar cada ciclo
const float TARGET_RAMP = 8.0;

// ======================================================
// ODOM
// ======================================================

float x = 0.0;
float y = 0.0;
float theta = 0.0;

// ======================================================
// ROS
// ======================================================

rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

rcl_subscription_t sub_cmd;
rcl_publisher_t pub_odom;

geometry_msgs__msg__Twist cmd_msg;
nav_msgs__msg__Odometry odom_msg;

// ======================================================
// WATCHDOG
// ======================================================

unsigned long last_cmd_time = 0;

const unsigned long CMD_TIMEOUT = 900;

// ======================================================
// ISR RIGHT
// ======================================================

// SOLO CANAL A
// MÁS ESTABLE

void encR_ISR()
{
  if (digitalRead(ENC_R_A) ==
      digitalRead(ENC_R_B))
  {
    encR++;
  }
  else
  {
    encR--;
  }
}

// ======================================================
// ISR LEFT
// ======================================================

void encL_ISR()
{
  if (digitalRead(ENC_L_A) ==
      digitalRead(ENC_L_B))
  {
    encL++;
  }
  else
  {
    encL--;
  }
}

// ======================================================
// CMD_VEL CALLBACK
// ======================================================

void cmdVelCallback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg =
    (const geometry_msgs__msg__Twist *)msgin;

  last_cmd_time = millis();

  //------------------------------------------
  // LIMITS
  //------------------------------------------

  float linear =
    constrain(
      msg->linear.x,
      -0.40,
      0.40);

  float angular =
    constrain(
      msg->angular.z,
      -2.5,
      2.5);

  //------------------------------------------
  // DIFFERENTIAL DRIVE
  //------------------------------------------

  float vR =
    linear +
    (WHEEL_BASE / 2.0) * angular;

  float vL =
    linear -
    (WHEEL_BASE / 2.0) * angular;

  //------------------------------------------
  // M/S -> RPM
  //------------------------------------------

  targetRPM_R =
    (vR * 60.0) /
    (2.0 * PI * WHEEL_RADIUS);

  targetRPM_L =
    (vL * 60.0) /
    (2.0 * PI * WHEEL_RADIUS);
}

// ======================================================
// MOTOR DRIVER
// ======================================================

void driveMotor(
  int pwm,
  int in1,
  int in2,
  int channel,
  bool invert)
{
  //------------------------------------------
  // DIRECTION
  //------------------------------------------

  bool dir = pwm >= 0;

  if (invert)
    dir = !dir;

  //------------------------------------------
  // PWM VALUE
  //------------------------------------------

  int value = abs(pwm);

  //------------------------------------------
  // DEADZONE
  //------------------------------------------

  if (value > 0 && value < MIN_PWM)
    value = MIN_PWM;

  //------------------------------------------

  value =
    constrain(
      value,
      0,
      MAX_PWM);

  //------------------------------------------
  // DIRECTION
  //------------------------------------------

  if (dir)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  //------------------------------------------
  // PWM
  //------------------------------------------

  ledcWrite(channel, value);
}

// ======================================================
// TARGET RAMP FUNCTION
// ======================================================

float rampTarget(
  float current,
  float target,
  float step)
{
  if (current < target)
  {
    current += step;

    if (current > target)
      current = target;
  }
  else if (current > target)
  {
    current -= step;

    if (current < target)
      current = target;
  }

  return current;
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(115200);

  //------------------------------------------
  // micro-ROS
  //------------------------------------------

  set_microros_transports();

  delay(2000);

  //------------------------------------------
  // MOTOR DRIVER
  //------------------------------------------

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  //------------------------------------------
  // PWM
  //------------------------------------------

  ledcSetup(CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMB, CH_R);

  ledcSetup(CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, CH_L);

  //------------------------------------------
  // ENCODERS
  //------------------------------------------

  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);

  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(ENC_R_A),
    encR_ISR,
    CHANGE);

  attachInterrupt(
    digitalPinToInterrupt(ENC_L_A),
    encL_ISR,
    CHANGE);

  //------------------------------------------
  // ROS2
  //------------------------------------------

  allocator =
    rcl_get_default_allocator();

  rclc_support_init(
    &support,
    0,
    NULL,
    &allocator);

  //------------------------------------------

  rclc_node_init_default(
    &node,
    "esp32_robot",
    "",
    &support);

  //------------------------------------------
  // SUBSCRIBER
  //------------------------------------------

  rclc_subscription_init_best_effort(
    &sub_cmd,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      geometry_msgs,
      msg,
      Twist),
    "/cmd_vel");

  //------------------------------------------
  // PUBLISHER
  //------------------------------------------

  rclc_publisher_init_default(
    &pub_odom,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      nav_msgs,
      msg,
      Odometry),
    "/odom");

  //------------------------------------------
  // EXECUTOR
  //------------------------------------------

  rclc_executor_init(
    &executor,
    &support.context,
    1,
    &allocator);

  //------------------------------------------

  rclc_executor_add_subscription(
    &executor,
    &sub_cmd,
    &cmd_msg,
    &cmdVelCallback,
    ON_NEW_DATA);

  //------------------------------------------

  last_cmd_time = millis();

  Serial.println("ETAPA 2 ESTABLE");
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  //------------------------------------------
  // ROS
  //------------------------------------------

  rclc_executor_spin_some(
    &executor,
    RCL_MS_TO_NS(10));

  //------------------------------------------
  // CONTROL LOOP
  //------------------------------------------

  static unsigned long last_time = millis();

  if (millis() - last_time >= 20)
  {
    float dt =
      (millis() - last_time) / 1000.0;

    last_time = millis();

    //--------------------------------------
    // WATCHDOG
    //--------------------------------------

    if (millis() - last_cmd_time >
        CMD_TIMEOUT)
    {
      targetRPM_R = 0;
      targetRPM_L = 0;
    }

    //--------------------------------------
    // TARGET RAMP
    //--------------------------------------

    smoothTargetR =
      rampTarget(
        smoothTargetR,
        targetRPM_R,
        TARGET_RAMP);

    smoothTargetL =
      rampTarget(
        smoothTargetL,
        targetRPM_L,
        TARGET_RAMP);

    //--------------------------------------
    // READ ENCODERS
    //--------------------------------------

    static long prevR = 0;
    static long prevL = 0;

    long currentR = encR;
    long currentL = encL;

    long deltaR = currentR - prevR;
    long deltaL = currentL - prevL;

    prevR = currentR;
    prevL = currentL;

    //--------------------------------------
    // RPM
    //--------------------------------------

    rpmR =
      (deltaR / PPR) *
      (60.0 / dt);

    rpmL =
      (deltaL / PPR) *
      (60.0 / dt);

    //--------------------------------------
    // FIX LEFT ENCODER
    //--------------------------------------

    rpmL = -rpmL;

    //--------------------------------------
    // FILTER
    //--------------------------------------

    rpmR_f =
      alpha * rpmR +
      (1.0 - alpha) * rpmR_f;

    rpmL_f =
      alpha * rpmL +
      (1.0 - alpha) * rpmL_f;

    //--------------------------------------
    // RIGHT PID
    //--------------------------------------

    if (abs(smoothTargetR) < 1.0)
    {
      pwmR = 0;

      iR = 0;
      eR_prev = 0;
    }
    else
    {
      eR =
        smoothTargetR -
        rpmR_f;

      iR += eR * dt;

      iR =
        constrain(iR, -80, 80);

      float dR =
        (eR - eR_prev) / dt;

      eR_prev = eR;

      pwmR =
        kp * eR +
        ki * iR +
        kd * dR;
    }

    //--------------------------------------
    // LEFT PID
    //--------------------------------------

    if (abs(smoothTargetL) < 1.0)
    {
      pwmL = 0;

      iL = 0;
      eL_prev = 0;
    }
    else
    {
      eL =
        smoothTargetL -
        rpmL_f;

      iL += eL * dt;

      iL =
        constrain(iL, -80, 80);

      float dL =
        (eL - eL_prev) / dt;

      eL_prev = eL;

      pwmL =
        kp * eL +
        ki * iL +
        kd * dL;
    }

    //--------------------------------------
    // LIMIT PWM
    //--------------------------------------

    pwmR =
      constrain(
        pwmR,
        -MAX_PWM,
        MAX_PWM);

    pwmL =
      constrain(
        pwmL,
        -MAX_PWM,
        MAX_PWM);

    //--------------------------------------
    // DRIVE
    //--------------------------------------

    driveMotor(
      pwmR,
      BIN1,
      BIN2,
      CH_R,
      INVERT_RIGHT);

    driveMotor(
      pwmL,
      AIN1,
      AIN2,
      CH_L,
      INVERT_LEFT);

    //--------------------------------------
    // ODOM
    //--------------------------------------

    float vR =
      (rpmR_f * 2.0 * PI * WHEEL_RADIUS)
      / 60.0;

    float vL =
      (rpmL_f * 2.0 * PI * WHEEL_RADIUS)
      / 60.0;

    float linear =
      (vR + vL) / 2.0;

    float angular =
      (vR - vL) / WHEEL_BASE;

    theta += angular * dt;

    x += linear * cos(theta) * dt;
    y += linear * sin(theta) * dt;

    //--------------------------------------
    // ODOM MSG
    //--------------------------------------

    odom_msg.header.frame_id.data =
      (char*)"odom";

    odom_msg.child_frame_id.data =
      (char*)"base_link";

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;

    //--------------------------------------
    // QUATERNION
    //--------------------------------------

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;

    odom_msg.pose.pose.orientation.z =
      sin(theta / 2.0);

    odom_msg.pose.pose.orientation.w =
      cos(theta / 2.0);

    //--------------------------------------

    odom_msg.twist.twist.linear.x =
      linear;

    odom_msg.twist.twist.angular.z =
      angular;

    //--------------------------------------
    // PUBLISH
    //--------------------------------------

    rcl_publish(
      &pub_odom,
      &odom_msg,
      NULL);
  }
}