#include <micro_ros_arduino.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <math.h>

// ================= MOTORS =================
#define PWMB 26
#define BIN1 27
#define BIN2 14

#define PWMA 32
#define AIN1 33
#define AIN2 25

#define STBY 13

// ================= ENCODERS =================
#define ENC_R_A 34
#define ENC_R_B 35
#define ENC_L_A 39
#define ENC_L_B 36

volatile long encR = 0;
volatile long encL = 0;

// ================= PWM =================
#define PWM_FREQ 5000
#define PWM_RES 8
#define CH_R 0
#define CH_L 1

// ================= ROBOT =================
const float WHEEL_RADIUS = 0.03235;
const float WHEEL_BASE   = 0.18;
const float PPR = 4230.0;

// ================= CONTROL =================
float targetRPM_R = 0;
float targetRPM_L = 0;

float rpmR = 0, rpmL = 0;
float rpmR_f = 0, rpmL_f = 0;

float alpha = 0.2;

// PID
float kp = 2.5, ki = 2.0, kd = 1.2;
float eR, eL, eR_prev, eL_prev;
float iR = 0, iL = 0;

int pwmR = 0, pwmL = 0;

// ================= ODOMETRY =================
float x = 0.0;
float y = 0.0;
float theta = 0.0;

// ================= ROS =================
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

rcl_subscription_t sub;
rcl_publisher_t pub_odom;

geometry_msgs__msg__Twist cmd_msg;
nav_msgs__msg__Odometry odom_msg;

// ================= CMD_VEL =================
void cmdVelCallback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg =
    (const geometry_msgs__msg__Twist *)msgin;

  float v = msg->linear.x;
  float w = msg->angular.z;

  float vR = v + (WHEEL_BASE/2.0)*w;
  float vL = v - (WHEEL_BASE/2.0)*w;

  targetRPM_R = (vR * 60.0) / (2.0 * PI * WHEEL_RADIUS);
  targetRPM_L = (vL * 60.0) / (2.0 * PI * WHEEL_RADIUS);
}

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);

  set_microros_transports();
  delay(2000);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  ledcSetup(CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMB, CH_R);

  ledcSetup(CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, CH_L);

  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);
  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_R_A), encR_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), encR_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), encL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), encL_ISR, CHANGE);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "esp32_odom", "", &support);

  // ================= SUB CMD_VEL =================
  rclc_subscription_init_best_effort(
    &sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel");

  // ================= PUB ODOM =================
  rclc_publisher_init_default(
    &pub_odom,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &sub, &cmd_msg, &cmdVelCallback, ON_NEW_DATA);

  Serial.println("ETAPA 2 READY");
}

// ================= LOOP =================
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  static long lastR = 0, lastL = 0;
  static unsigned long lastT = millis();

  if (millis() - lastT >= 20)
  {
    float dt = (millis() - lastT) / 1000.0;
    lastT = millis();

    // ================= ENCODERS =================
    long r = encR;
    long l = encL;

    rpmR = ((r - lastR)/PPR) * (60.0/dt);
    rpmL = ((l - lastL)/PPR) * (60.0/dt);

    lastR = r;
    lastL = l;

    rpmR_f = alpha*rpmR + (1-alpha)*rpmR_f;
    rpmL_f = alpha*rpmL + (1-alpha)*rpmL_f;

    // ================= PID =================
    eR = targetRPM_R - rpmR_f;
    iR += eR * dt;
    float dR = (eR - eR_prev)/dt;
    eR_prev = eR;

    pwmR = kp*eR + ki*iR + kd*dR;

    eL = targetRPM_L - rpmL_f;
    iL += eL * dt;
    float dL = (eL - eL_prev)/dt;
    eL_prev = eL;

    pwmL = kp*eL + ki*iL + kd*dL;

    driveMotor(pwmR, PWMB, BIN1, BIN2, CH_R);
    driveMotor(pwmL, PWMA, AIN1, AIN2, CH_L);

    // ================= ODOMETRY =================
    float vR = (rpmR_f * 2.0 * PI * WHEEL_RADIUS) / 60.0;
    float vL = (rpmL_f * 2.0 * PI * WHEEL_RADIUS) / 60.0;

    float v = (vR + vL) / 2.0;
    float w = (vR - vL) / WHEEL_BASE;

    theta += w * dt;
    x += v * cos(theta) * dt;
    y += v * sin(theta) * dt;

    // ================= PUBLISH ODOM =================
    publishOdom(v, w);
  }
}

// ================= ODOM PUBLISH =================
void publishOdom(float v, float w)
{
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;

  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.angular.z = w;

  rcl_publish(&pub_odom, &odom_msg, NULL);
}

// ================= MOTOR =================
void driveMotor(int pwm, int pinPWM, int in1, int in2, int ch)
{
  bool dir = pwm >= 0;
  int val = abs(pwm);

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

  ledcWrite(ch, constrain(val, 0, 220));
}

// ================= ISR =================
void encR_ISR()
{
  bool A = digitalRead(ENC_R_A);
  bool B = digitalRead(ENC_R_B);
  encR += (A == B) ? 1 : -1;
}

void encL_ISR()
{
  bool A = digitalRead(ENC_L_A);
  bool B = digitalRead(ENC_L_B);
  encL += (A == B) ? 1 : -1;
}