//
//==================================================
// ROBOT DIFERENCIAL FINAL
// PID + ENCODERS + ODOM + IMU
// ESP32 + TB6612FNG + MPU6050 + DHT22
//==================================================
//

#include <math.h>
#include <Wire.h>
#include <DHT.h>

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
// MPU6050
//==================================================
//

#define MPU6050_ADDR 0x68

//
//==================================================
// DHT22
//==================================================
//

#define DHTPIN 19
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

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

float alphaRPM = 0.197;

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
// MPU RAW
//==================================================
//

int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

//
//==================================================
// IMU REAL
//==================================================
//

float ax, ay, az;
float gx, gy, gz;

//
//==================================================
// FILTERED IMU
//==================================================
//

float ax_f = 0;
float ay_f = 0;
float az_f = 0;

float gx_f = 0;
float gy_f = 0;
float gz_f = 0;

//
//==================================================
// GYRO OFFSETS
//==================================================
//

float gx_offset = 0;
float gy_offset = 0;
float gz_offset = 0;

//
//==================================================
// ORIENTATION
//==================================================
//

float roll = 0;
float pitch = 0;
float yaw = 0;

float yaw_filtered = 0;

//
//==================================================
// FILTERS
//==================================================
//

float alphaIMU = 0.95;

float accelLowPass = 0.97;
float gyroLowPass  = 0.88;

float yawAlpha = 0.97;

//
//==================================================
// DHT
//==================================================
//

float humidity = 0;
float temperature = 0;

//
//==================================================
// TIME
//==================================================
//

unsigned long prevTime = 0;
float dtIMU = 0;

//
//==================================================
// SETUP
//==================================================
//

void setup() {

  Serial.begin(115200);

  //--------------------------------
  // ROBOT
  //--------------------------------

  setupMotors();

  setupPWM();

  setupEncoders();

  //--------------------------------
  // I2C
  //--------------------------------

  Wire.begin(22, 21);

  Wire.setClock(400000);

  //--------------------------------
  // DHT22
  //--------------------------------

  dht.begin();

  //--------------------------------
  // MPU6050
  //--------------------------------

  setupMPU();

  delay(1000);

  calibrateGyro();

  prevTime = micros();

  Serial.println("SYSTEM READY");
}

//
//==================================================
// LOOP
//==================================================
//

void loop() {

  static unsigned long lastControl = 0;

  //--------------------------------
  // IMU DT
  //--------------------------------

  updateDeltaTime();

  //--------------------------------
  // IMU
  //--------------------------------

  readMPU();

  processIMU();

  //--------------------------------
  // CONTROL LOOP
  //--------------------------------

  if (millis() - lastControl >= 100) {

    //--------------------------------
    // RPM
    //--------------------------------

    updateRPM();

    applyRPMFilter();

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
    // IMU + ODOM FUSION
    //--------------------------------

    fuseIMUWithOdometry();

    //--------------------------------
    // DHT22
    //--------------------------------

    readDHT();

    //--------------------------------
    // SERIAL
    //--------------------------------

    printRobotState();

    lastControl = millis();
  }
}

//
//==================================================
// MOTOR SETUP
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

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

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

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_RIGHT_A),
      handleRightEncoderA,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_RIGHT_B),
      handleRightEncoderB,
      CHANGE);

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
// MPU SETUP
//==================================================
//

void setupMPU() {

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  //--------------------------------

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  //--------------------------------

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  //--------------------------------

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);
}

//
//==================================================
// UPDATE DT
//==================================================
//

void updateDeltaTime() {

  unsigned long currentTime = micros();

  dtIMU =
      (currentTime - prevTime) /
      1000000.0;

  prevTime = currentTime;
}

//
//==================================================
// READ MPU
//==================================================
//

void readMPU() {

  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(0x3B);

  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050_ADDR, 14, true);

  //--------------------------------

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  //--------------------------------

  Wire.read();
  Wire.read();

  //--------------------------------

  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

//
//==================================================
// PROCESS IMU
//==================================================
//

void processIMU() {

  //--------------------------------

  ax = AcX / 16384.0;
  ay = AcY / 16384.0;
  az = AcZ / 16384.0;

  //--------------------------------

  gx =
      (GyX - gx_offset) /
      131.0;

  gy =
      (GyY - gy_offset) /
      131.0;

  gz =
      (GyZ - gz_offset) /
      131.0;

  //--------------------------------
  // LOW PASS
  //--------------------------------

  ax_f =
      accelLowPass * ax_f +
      (1 - accelLowPass) * ax;

  ay_f =
      accelLowPass * ay_f +
      (1 - accelLowPass) * ay;

  az_f =
      accelLowPass * az_f +
      (1 - accelLowPass) * az;

  //--------------------------------

  gx_f =
      gyroLowPass * gx_f +
      (1 - gyroLowPass) * gx;

  gy_f =
      gyroLowPass * gy_f +
      (1 - gyroLowPass) * gy;

  gz_f =
      gyroLowPass * gz_f +
      (1 - gyroLowPass) * gz;

  //--------------------------------
  // ACCEL ANGLES
  //--------------------------------

  float accelRoll =
      atan2(ay_f, az_f) *
      180.0 / PI;

  //--------------------------------

  float accelPitch =
      atan2(
          -ax_f,
          sqrt(
              ay_f * ay_f +
              az_f * az_f))
      * 180.0 / PI;

  //--------------------------------
  // GYRO INTEGRATION
  //--------------------------------

  roll += gx_f * dtIMU;
  pitch += gy_f * dtIMU;
  yaw += gz_f * dtIMU;

  //--------------------------------
  // COMPLEMENTARY FILTER
  //--------------------------------

  roll =
      alphaIMU * roll +
      (1 - alphaIMU) * accelRoll;

  //--------------------------------

  pitch =
      alphaIMU * pitch +
      (1 - alphaIMU) * accelPitch;

  //--------------------------------
  // YAW FILTER
  //--------------------------------

  yaw_filtered =
      yawAlpha * yaw_filtered +
      (1 - yawAlpha) * yaw;
}

//
//==================================================
// CALIBRATE GYRO
//==================================================
//

void calibrateGyro() {

  long gx_sum = 0;
  long gy_sum = 0;
  long gz_sum = 0;

  const int samples = 5000;

  Serial.println("KEEP ROBOT STILL");

  delay(3000);

  for (int i = 0; i < samples; i++) {

    readRawGyro();

    gx_sum += GyX;
    gy_sum += GyY;
    gz_sum += GyZ;

    delay(2);
  }

  //--------------------------------

  gx_offset =
      gx_sum / (float)samples;

  gy_offset =
      gy_sum / (float)samples;

  gz_offset =
      gz_sum / (float)samples;

  Serial.println("GYRO CALIBRATED");
}

//
//==================================================
// RAW GYRO
//==================================================
//

void readRawGyro() {

  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(0x43);

  Wire.endTransmission(false);

  Wire.requestFrom(
      MPU6050_ADDR,
      6,
      true);

  //--------------------------------

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

  long deltaRight =
      rightPos - lastRight;

  long deltaLeft =
      leftPos - lastLeft;

  //--------------------------------

  rpmRight_raw =
      (deltaRight / PPR) *
      (60000.0 / 100.0);

  //--------------------------------

  rpmLeft_raw =
      (deltaLeft / PPR) *
      (60000.0 / 100.0);

  //--------------------------------

  rpmLeft_raw = -rpmLeft_raw;

  //--------------------------------

  lastRight = rightPos;
  lastLeft  = leftPos;
}

//
//==================================================
// RPM FILTER
//==================================================
//

void applyRPMFilter() {

  rpmRight_ema =
      alphaRPM * rpmRight_raw +
      (1 - alphaRPM) * rpmRight_ema;

  //--------------------------------

  rpmLeft_ema =
      alphaRPM * rpmLeft_raw +
      (1 - alphaRPM) * rpmLeft_ema;
}

//
//==================================================
// PID RIGHT
//==================================================
//

void updatePIDRight() {

  errorRight =
      setpointRight -
      rpmRight_ema;

  //--------------------------------

  integralRight +=
      errorRight * DT;

  //--------------------------------

  float derivative =
      (errorRight -
       prevErrorRight) / DT;

  //--------------------------------

  float output =
      kpRight * errorRight +
      kiRight * integralRight +
      kdRight * derivative;

  //--------------------------------

  pwmRight += (int)output;

  //--------------------------------

  pwmRight =
      constrain(
          pwmRight,
          0,
          225);

  //--------------------------------

  prevErrorRight =
      errorRight;
}

//
//==================================================
// PID LEFT
//==================================================
//

void updatePIDLeft() {

  errorLeft =
      setpointLeft -
      rpmLeft_ema;

  //--------------------------------

  integralLeft +=
      errorLeft * DT;

  //--------------------------------

  float derivative =
      (errorLeft -
       prevErrorLeft) / DT;

  //--------------------------------

  float output =
      kpLeft * errorLeft +
      kiLeft * integralLeft +
      kdLeft * derivative;

  //--------------------------------

  pwmLeft += (int)output;

  //--------------------------------

  pwmLeft =
      constrain(
          pwmLeft,
          0,
          225);

  //--------------------------------

  prevErrorLeft =
      errorLeft;
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

  float omegaRight =
      (2.0 * PI *
       rpmRight_ema) / 60.0;

  //--------------------------------

  float omegaLeft =
      (2.0 * PI *
       rpmLeft_ema) / 60.0;

  //--------------------------------

  float vRight =
      WHEEL_RADIUS *
      omegaRight;

  //--------------------------------

  float vLeft =
      WHEEL_RADIUS *
      omegaLeft;

  //--------------------------------

  linearVelocity =
      (vRight + vLeft) / 2.0;

  //--------------------------------

  angularVelocity =
      (vRight - vLeft) /
      WHEEL_BASE;

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
// IMU + ODOM FUSION
//==================================================
//

void fuseIMUWithOdometry() {

  float imuYawRad =
      radians(yaw_filtered);

  //--------------------------------

  theta =
      0.98 * theta +
      0.02 * imuYawRad;
}

//
//==================================================
// DHT22
//==================================================
//

void readDHT() {

  humidity =
      dht.readHumidity();

  //--------------------------------

  temperature =
      dht.readTemperature();
}

//
//==================================================
// SERIAL
//==================================================
//

void printRobotState() {

  Serial.print("X:");
  Serial.print(x);

  Serial.print(",");

  Serial.print("Y:");
  Serial.print(y);

  Serial.print(",");

  Serial.print("TH:");
  Serial.print(theta);

  Serial.print(",");

  Serial.print("RR:");
  Serial.print(rpmRight_ema);

  Serial.print(",");

  Serial.print("RL:");
  Serial.print(rpmLeft_ema);

  Serial.print(",");

  Serial.print("YAW:");
  Serial.print(yaw_filtered);

  Serial.print(",");

  Serial.print("TEMP:");
  Serial.print(temperature);

  Serial.print(",");

  Serial.print("HUM:");
  Serial.println(humidity);
}

//
//==================================================
// ISR RIGHT
//==================================================
//

void handleRightEncoderA() {

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

void handleRightEncoderB() {

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

void handleLeftEncoderA() {

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

void handleLeftEncoderB() {

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