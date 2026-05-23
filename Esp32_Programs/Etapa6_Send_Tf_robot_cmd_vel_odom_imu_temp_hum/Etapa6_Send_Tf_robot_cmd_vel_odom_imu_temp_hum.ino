// ======================================================
// STAGE 5.1
// micro-ROS + TF + ODOM + IMU + DHT22
// ENCODER + IMU YAW FUSION
// ======================================================

#include <micro_ros_arduino.h>

#include <Wire.h>
#include <DHT.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <nav_msgs/msg/odometry.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/relative_humidity.h>

#include <tf2_msgs/msg/tf_message.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <math.h>

// ======================================================
// DHT22
// ======================================================

#define DHTPIN 19
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

// ======================================================
// MPU6050
// ======================================================

#define MPU6050_ADDR 0x68

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

float smoothTargetR = 0.0;
float smoothTargetL = 0.0;

float rpmR = 0.0;
float rpmL = 0.0;

float rpmR_f = 0.0;
float rpmL_f = 0.0;

float alphaRPM = 0.25;

// ======================================================
// PID
// ======================================================

float kp = 1.8;
float ki = 0.8;
float kd = 0.15;

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
// RAMP
// ======================================================

const float TARGET_RAMP = 8.0;

// ======================================================
// ODOM
// ======================================================

float x = 0.0;
float y = 0.0;
float theta = 0.0;

// ======================================================
// YAW FUSION
// ======================================================

float theta_encoder = 0.0;
float theta_imu     = 0.0;

float fusionAlpha = 0.98;

// ======================================================
// IMU RAW
// ======================================================

int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

// ======================================================
// IMU REAL
// ======================================================

float ax, ay, az;
float gx, gy, gz;

// ======================================================
// FILTERED
// ======================================================

float ax_f = 0;
float ay_f = 0;
float az_f = 0;

float gx_f = 0;
float gy_f = 0;
float gz_f = 0;

// ======================================================
// OFFSETS
// ======================================================

float gx_offset = 0;
float gy_offset = 0;
float gz_offset = 0;

// ======================================================
// ORIENTATION
// ======================================================

float roll = 0;
float pitch = 0;
float yaw = 0;

float yaw_filtered = 0;

// ======================================================
// FILTERS
// ======================================================

float alphaIMU = 0.95;

float accelLowPass = 0.97;
float gyroLowPass  = 0.88;

float yawAlpha = 0.97;

// ======================================================
// DHT
// ======================================================

float humidity = 0;
float temperature = 0;

// ======================================================
// ROS
// ======================================================

rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

rcl_subscription_t sub_cmd;

rcl_publisher_t pub_odom;
rcl_publisher_t pub_imu;
rcl_publisher_t pub_temp;
rcl_publisher_t pub_humidity;
rcl_publisher_t pub_tf;

geometry_msgs__msg__Twist cmd_msg;

nav_msgs__msg__Odometry odom_msg;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Temperature temp_msg;
sensor_msgs__msg__RelativeHumidity humidity_msg;

tf2_msgs__msg__TFMessage tf_msg;

geometry_msgs__msg__TransformStamped tf_transform;

// ======================================================
// WATCHDOG
// ======================================================

unsigned long last_cmd_time = 0;

const unsigned long CMD_TIMEOUT = 900;

// ======================================================
// ISR RIGHT
// ======================================================

void encR_ISR()
{
  if (digitalRead(ENC_R_A) ==
      digitalRead(ENC_R_B))
    encR++;
  else
    encR--;
}

// ======================================================
// ISR LEFT
// ======================================================

void encL_ISR()
{
  if (digitalRead(ENC_L_A) ==
      digitalRead(ENC_L_B))
    encL++;
  else
    encL--;
}

// ======================================================
// CMD CALLBACK
// ======================================================

void cmdVelCallback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg =
    (const geometry_msgs__msg__Twist *)msgin;

  last_cmd_time = millis();

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

  float vR =
    linear +
    (WHEEL_BASE / 2.0) * angular;

  float vL =
    linear -
    (WHEEL_BASE / 2.0) * angular;

  targetRPM_R =
    (vR * 60.0) /
    (2.0 * PI * WHEEL_RADIUS);

  targetRPM_L =
    (vL * 60.0) /
    (2.0 * PI * WHEEL_RADIUS);
}

// ======================================================
// MOTOR
// ======================================================

void driveMotor(
  int pwm,
  int in1,
  int in2,
  int channel,
  bool invert)
{
  bool dir = pwm >= 0;

  if (invert)
    dir = !dir;

  int value = abs(pwm);

  if (value > 0 &&
      value < MIN_PWM)
  {
    value = MIN_PWM;
  }

  value =
    constrain(
      value,
      0,
      MAX_PWM);

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

  ledcWrite(channel, value);
}

// ======================================================
// RAMP
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
// MPU
// ======================================================

void setupI2C()
{
  Wire.begin(22, 21);
  Wire.setClock(400000);
}

void setupMPU()
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);
}

void readMPU()
{
  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(0x3B);

  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050_ADDR, 14, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  Wire.read();
  Wire.read();

  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

void convertRawData()
{
  ax = AcX / 16384.0;
  ay = AcY / 16384.0;
  az = AcZ / 16384.0;

  gx = (GyX - gx_offset) / 131.0;
  gy = (GyY - gy_offset) / 131.0;
  gz = (GyZ - gz_offset) / 131.0;
}

void applyFilters()
{
  ax_f =
    accelLowPass * ax_f +
    (1 - accelLowPass) * ax;

  ay_f =
    accelLowPass * ay_f +
    (1 - accelLowPass) * ay;

  az_f =
    accelLowPass * az_f +
    (1 - accelLowPass) * az;

  gx_f =
    gyroLowPass * gx_f +
    (1 - gyroLowPass) * gx;

  gy_f =
    gyroLowPass * gy_f +
    (1 - gyroLowPass) * gy;

  gz_f =
    gyroLowPass * gz_f +
    (1 - gyroLowPass) * gz;
}

void integrateGyro(float dt)
{
  roll += gx_f * dt;
  pitch += gy_f * dt;
  yaw += gz_f * dt;
}

void applyComplementaryFilter(
  float accelRoll,
  float accelPitch)
{
  roll =
    alphaIMU * roll +
    (1 - alphaIMU) * accelRoll;

  pitch =
    alphaIMU * pitch +
    (1 - alphaIMU) * accelPitch;
}

void filterYaw()
{
  yaw_filtered =
    yawAlpha * yaw_filtered +
    (1 - yawAlpha) * yaw;
}

void processIMU(float dt)
{
  convertRawData();

  applyFilters();

  float accelRoll =
    atan2(ay_f, az_f) *
    180.0 / PI;

  float accelPitch =
    atan2(
      -ax_f,
      sqrt(
        ay_f * ay_f +
        az_f * az_f))
    * 180.0 / PI;

  integrateGyro(dt);

  applyComplementaryFilter(
    accelRoll,
    accelPitch);

  filterYaw();
}

void readRawGyro()
{
  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(0x43);

  Wire.endTransmission(false);

  Wire.requestFrom(
    MPU6050_ADDR,
    6,
    true);

  GyX =
    Wire.read() << 8 |
    Wire.read();

  GyY =
    Wire.read() << 8 |
    Wire.read();

  GyZ =
    Wire.read() << 8 |
    Wire.read();
}

void calibrateGyro()
{
  long gx_sum = 0;
  long gy_sum = 0;
  long gz_sum = 0;

  const int samples = 2000;

  Serial.println("KEEP ROBOT STILL");

  delay(3000);

  for (int i = 0; i < samples; i++)
  {
    readRawGyro();

    gx_sum += GyX;
    gy_sum += GyY;
    gz_sum += GyZ;

    delay(2);
  }

  gx_offset =
    gx_sum / (float)samples;

  gy_offset =
    gy_sum / (float)samples;

  gz_offset =
    gz_sum / (float)samples;

  Serial.println("GYRO READY");
}

// ======================================================
// DHT
// ======================================================

void setupDHT()
{
  dht.begin();
}

void readDHT()
{
  humidity =
    dht.readHumidity();

  temperature =
    dht.readTemperature();
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(115200);

  setupI2C();

  setupDHT();

  setupMPU();

  calibrateGyro();

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

  attachInterrupt(
    digitalPinToInterrupt(ENC_R_A),
    encR_ISR,
    CHANGE);

  attachInterrupt(
    digitalPinToInterrupt(ENC_L_A),
    encL_ISR,
    CHANGE);

  allocator =
    rcl_get_default_allocator();

  rclc_support_init(
    &support,
    0,
    NULL,
    &allocator);

  rclc_node_init_default(
    &node,
    "esp32_robot",
    "",
    &support);

  rclc_subscription_init_best_effort(
    &sub_cmd,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      geometry_msgs,
      msg,
      Twist),
    "/cmd_vel");

  rclc_publisher_init_best_effort(
    &pub_odom,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      nav_msgs,
      msg,
      Odometry),
    "/odom");

  rclc_publisher_init_best_effort(
    &pub_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      sensor_msgs,
      msg,
      Imu),
    "/imu");

  rclc_publisher_init_best_effort(
    &pub_temp,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      sensor_msgs,
      msg,
      Temperature),
    "/temperature");

  rclc_publisher_init_best_effort(
    &pub_humidity,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      sensor_msgs,
      msg,
      RelativeHumidity),
    "/humidity");

  rclc_publisher_init_default(
    &pub_tf,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      tf2_msgs,
      msg,
      TFMessage),
    "/tf");

  rclc_executor_init(
    &executor,
    &support.context,
    1,
    &allocator);

  rclc_executor_add_subscription(
    &executor,
    &sub_cmd,
    &cmd_msg,
    &cmdVelCallback,
    ON_NEW_DATA);

  last_cmd_time = millis();

  Serial.println("STAGE 5.1 READY");
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  rclc_executor_spin_some(
    &executor,
    RCL_MS_TO_NS(1));

  static unsigned long lastControl = millis();

  if (millis() - lastControl >= 20)
  {
    float dt =
      (millis() - lastControl)
      / 1000.0;

    lastControl = millis();

    // =====================================
    // WATCHDOG
    // =====================================

    if (millis() - last_cmd_time >
        CMD_TIMEOUT)
    {
      targetRPM_R = 0;
      targetRPM_L = 0;

      iR = 0;
      iL = 0;
    }

    // =====================================
    // RAMP
    // =====================================

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

    // =====================================
    // ENCODERS
    // =====================================

    static long prevR = 0;
    static long prevL = 0;

    long currentR = encR;
    long currentL = encL;

    long deltaR =
      currentR - prevR;

    long deltaL =
      currentL - prevL;

    prevR = currentR;
    prevL = currentL;

    // =====================================
    // RPM
    // =====================================

    rpmR =
      (deltaR / PPR) *
      (60.0 / dt);

    rpmL =
      (deltaL / PPR) *
      (60.0 / dt);

    rpmL = -rpmL;

    // =====================================
    // FILTER RPM
    // =====================================

    rpmR_f =
      alphaRPM * rpmR +
      (1.0 - alphaRPM) *
      rpmR_f;

    rpmL_f =
      alphaRPM * rpmL +
      (1.0 - alphaRPM) *
      rpmL_f;

    // =====================================
    // PID RIGHT
    // =====================================

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

    // =====================================
    // PID LEFT
    // =====================================

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

    // =====================================
    // DRIVE
    // =====================================

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

    // =====================================
    // IMU
    // =====================================

    readMPU();

    processIMU(dt);

    // =====================================
    // VELOCITIES
    // =====================================

    float vR =
      (rpmR_f *
       2.0 *
       PI *
       WHEEL_RADIUS)
      / 60.0;

    float vL =
      (rpmL_f *
       2.0 *
       PI *
       WHEEL_RADIUS)
      / 60.0;

    float linear =
      (vR + vL) / 2.0;

    float angular_encoder =
      (vR - vL) / WHEEL_BASE;

    float angular_imu =
      gz_f * PI / 180.0;

    theta_encoder +=
      angular_encoder * dt;

    theta_imu +=
      angular_imu * dt;

    theta =
      fusionAlpha *
      theta_imu +

      (1.0 - fusionAlpha) *
      theta_encoder;

    x +=
      linear *
      cos(theta) *
      dt;

    y +=
      linear *
      sin(theta) *
      dt;

    // =====================================
    // ODOM
    // =====================================

    odom_msg.header.frame_id.data =
      (char*)"odom";

    odom_msg.child_frame_id.data =
      (char*)"base_footprint";

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;

    odom_msg.pose.pose.orientation.z =
      sin(theta / 2.0);

    odom_msg.pose.pose.orientation.w =
      cos(theta / 2.0);

    odom_msg.twist.twist.linear.x =
      linear;

    odom_msg.twist.twist.angular.z =
      angular_imu;

    rcl_publish(
      &pub_odom,
      &odom_msg,
      NULL);

    // =====================================
    // TF
    // =====================================

    tf_transform.header.frame_id.data =
      (char*)"odom";

    tf_transform.child_frame_id.data =
      (char*)"base_footprint";

    tf_transform.transform.translation.x = x;
    tf_transform.transform.translation.y = y;
    tf_transform.transform.translation.z = 0.0;

    tf_transform.transform.rotation.x = 0.0;
    tf_transform.transform.rotation.y = 0.0;

    tf_transform.transform.rotation.z =
      sin(theta / 2.0);

    tf_transform.transform.rotation.w =
      cos(theta / 2.0);

    tf_msg.transforms.data =
      &tf_transform;

    tf_msg.transforms.size = 1;
    tf_msg.transforms.capacity = 1;

    rcl_publish(
      &pub_tf,
      &tf_msg,
      NULL);

    // =====================================
    // IMU MSG
    // =====================================

    imu_msg.header.frame_id.data =
      (char*)"imu_link";

    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;

    imu_msg.orientation.z =
      sin(theta / 2.0);

    imu_msg.orientation.w =
      cos(theta / 2.0);

    imu_msg.angular_velocity.x =
      gx_f * PI / 180.0;

    imu_msg.angular_velocity.y =
      gy_f * PI / 180.0;

    imu_msg.angular_velocity.z =
      angular_imu;

    imu_msg.linear_acceleration.x =
      ax_f * 9.81;

    imu_msg.linear_acceleration.y =
      ay_f * 9.81;

    imu_msg.linear_acceleration.z =
      az_f * 9.81;

    rcl_publish(
      &pub_imu,
      &imu_msg,
      NULL);
  }

  // =========================================
  // DHT LOOP
  // =========================================

  static unsigned long lastDHT = millis();

  if (millis() - lastDHT >= 1000)
  {
    lastDHT = millis();

    readDHT();

    temp_msg.header.frame_id.data =
      (char*)"base_link";

    temp_msg.temperature =
      temperature;

    humidity_msg.header.frame_id.data =
      (char*)"base_link";

    humidity_msg.relative_humidity =
      humidity / 100.0;

    rcl_publish(
      &pub_temp,
      &temp_msg,
      NULL);

    rcl_publish(
      &pub_humidity,
      &humidity_msg,
      NULL);
  }
}