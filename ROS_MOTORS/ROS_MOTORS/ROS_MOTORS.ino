#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/twist.h>

// ----------- RIGHT MOTOR -----------
#define PWMB 26
#define BIN1 27
#define BIN2 14

// ----------- LEFT MOTOR -----------
#define PWMA 32
#define AIN1 33
#define AIN2 25

#define STBY 12  // puedes cambiar si lo usas

// PWM
#define PWM_FREQ 1000
#define PWM_RES 8

#define CH_RIGHT 0
#define CH_LEFT  1

// micro-ROS
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// CONTROL
float max_pwm = 200.0;

// ---------- MOTOR ----------
void setMotor(int pwm, int in1, int in2, int channel) {
  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    pwm = -pwm;
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  ledcWrite(channel, pwm);
}

// ---------- CALLBACK ----------
void cmdVelCallback(const void * msgin) {
  const geometry_msgs__msg__Twist * cmd = (const geometry_msgs__msg__Twist *)msgin;

  float linear  = cmd->linear.x;
  float angular = cmd->angular.z;

  float left  = linear - angular;
  float right = linear + angular;

  int pwm_left  = (int)(left  * max_pwm);
  int pwm_right = (int)(right * max_pwm);

  pwm_left  = constrain(pwm_left,  -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);

  setMotor(pwm_left,  AIN1, AIN2, CH_LEFT);
  setMotor(pwm_right, BIN1, BIN2, CH_RIGHT);
}

// ---------- SETUP ----------
void setup() {
  set_microros_transports();

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  ledcSetup(CH_LEFT, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, CH_LEFT);

  ledcSetup(CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMB, CH_RIGHT);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);

  rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &cmdVelCallback,
    ON_NEW_DATA
  );
}

// ---------- LOOP ----------
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
