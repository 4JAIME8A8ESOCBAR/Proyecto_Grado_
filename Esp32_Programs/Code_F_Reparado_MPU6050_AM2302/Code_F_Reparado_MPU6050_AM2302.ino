#include <Wire.h>
#include <DHT.h>

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
// MPU6050
//==================================================
//

#define MPU6050_ADDR 0x68

//
//==================================================
// RAW DATA
//==================================================
//

int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

//
//==================================================
// ACCEL
//==================================================
//

float ax, ay, az;

//
//==================================================
// GYRO
//==================================================
//

float gx, gy, gz;

//
//==================================================
// FILTERED VALUES
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
// OFFSETS
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

float alpha = 0.95;

float accelLowPass = 0.97;
float gyroLowPass  = 0.88;

float yawAlpha = 0.97;

//
//==================================================
// TIME
//==================================================
//

unsigned long prevTime = 0;

float dt = 0;

//
//==================================================
// DHT VARIABLES
//==================================================
//

float humidity = 0;
float temperature = 0;

//
//==================================================
// SETUP
//==================================================
//

void setup() {

  Serial.begin(115200);

  setupI2C();

  setupDHT();

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

  //--------------------------------
  // TIME
  //--------------------------------

  updateDeltaTime();

  //--------------------------------
  // MPU6050
  //--------------------------------

  readMPU();

  processIMU();

  //--------------------------------
  // DHT22
  //--------------------------------

  readDHT();

  //--------------------------------
  // SERIAL
  //--------------------------------

  printAllData();
}

//
//==================================================
// SETUP I2C
//==================================================
//

void setupI2C() {

  Wire.begin(22, 21);

  Wire.setClock(400000);
}

//
//==================================================
// SETUP DHT
//==================================================
//

void setupDHT() {

  dht.begin();
}

//
//==================================================
// SETUP MPU6050
//==================================================
//

void setupMPU() {

  //--------------------------------
  // WAKE UP MPU
  //--------------------------------

  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(0x6B);

  Wire.write(0x00);

  Wire.endTransmission(true);

  //--------------------------------
  // GYRO ±250 deg/s
  //--------------------------------

  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(0x1B);

  Wire.write(0x00);

  Wire.endTransmission(true);

  //--------------------------------
  // ACCEL ±2g
  //--------------------------------

  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(0x1C);

  Wire.write(0x00);

  Wire.endTransmission(true);

  //--------------------------------
  // DLPF
  //--------------------------------

  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(0x1A);

  Wire.write(0x03);

  Wire.endTransmission(true);
}

//
//==================================================
// UPDATE DELTA TIME
//==================================================
//

void updateDeltaTime() {

  unsigned long currentTime = micros();

  dt =
      (currentTime - prevTime) /
      1000000.0;

  prevTime = currentTime;
}

//
//==================================================
// READ MPU6050
//==================================================
//

void readMPU() {

  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(0x3B);

  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050_ADDR, 14, true);

  //--------------------------------
  // ACCEL
  //--------------------------------

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  //--------------------------------
  // SKIP TEMP
  //--------------------------------

  Wire.read();
  Wire.read();

  //--------------------------------
  // GYRO
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
  // RAW TO REAL
  //--------------------------------

  convertRawData();

  //--------------------------------
  // LOW PASS FILTERS
  //--------------------------------

  applyFilters();

  //--------------------------------
  // ACCEL ANGLES
  //--------------------------------

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

  //--------------------------------
  // GYRO INTEGRATION
  //--------------------------------

  integrateGyro();

  //--------------------------------
  // COMPLEMENTARY FILTER
  //--------------------------------

  applyComplementaryFilter(
      accelRoll,
      accelPitch);

  //--------------------------------
  // YAW FILTER
  //--------------------------------

  filterYaw();
}

//
//==================================================
// CONVERT RAW DATA
//==================================================
//

void convertRawData() {

  //--------------------------------
  // ACCEL
  //--------------------------------

  ax = AcX / 16384.0;
  ay = AcY / 16384.0;
  az = AcZ / 16384.0;

  //--------------------------------
  // GYRO
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
}

//
//==================================================
// APPLY FILTERS
//==================================================
//

void applyFilters() {

  //--------------------------------
  // ACC FILTER
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
  // GYRO FILTER
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
}

//
//==================================================
// GYRO INTEGRATION
//==================================================
//

void integrateGyro() {

  roll += gx_f * dt;

  pitch += gy_f * dt;

  yaw += gz_f * dt;
}

//
//==================================================
// COMPLEMENTARY FILTER
//==================================================
//

void applyComplementaryFilter(
    float accelRoll,
    float accelPitch) {

  roll =
      alpha * roll +
      (1 - alpha) * accelRoll;

  pitch =
      alpha * pitch +
      (1 - alpha) * accelPitch;
}

//
//==================================================
// FILTER YAW
//==================================================
//

void filterYaw() {

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

  //--------------------------------
  // DISCARD INITIAL DATA
  //--------------------------------

  for (int i = 0; i < 200; i++) {

    readRawGyro();

    delay(2);
  }

  //--------------------------------
  // CALIBRATION
  //--------------------------------

  Serial.println("CALIBRATING GYRO...");

  for (int i = 0; i < samples; i++) {

    readRawGyro();

    gx_sum += GyX;
    gy_sum += GyY;
    gz_sum += GyZ;

    delay(2);
  }

  //--------------------------------
  // OFFSETS
  //--------------------------------

  gx_offset =
      gx_sum / (float)samples;

  gy_offset =
      gy_sum / (float)samples;

  gz_offset =
      gz_sum / (float)samples;

  //--------------------------------
  // RESULTS
  //--------------------------------

  Serial.println("GYRO CALIBRATED");

  Serial.print("GX OFFSET: ");
  Serial.println(gx_offset);

  Serial.print("GY OFFSET: ");
  Serial.println(gy_offset);

  Serial.print("GZ OFFSET: ");
  Serial.println(gz_offset);
}

//
//==================================================
// READ RAW GYRO
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
// READ DHT22
//==================================================
//

void readDHT() {

  humidity =
      dht.readHumidity();

  temperature =
      dht.readTemperature();
}

//
//==================================================
// PRINT ALL DATA
//==================================================
//

void printAllData() {

  //--------------------------------
  // ORIENTATION
  //--------------------------------

  Serial.print("ROLL:");
  Serial.print(roll);

  Serial.print(",");

  Serial.print("PITCH:");
  Serial.print(pitch);

  Serial.print(",");

  Serial.print("YAW:");
  Serial.print(yaw_filtered);

  Serial.print(",");

  //--------------------------------
  // GYRO
  //--------------------------------

  Serial.print("GX:");
  Serial.print(gx_f);

  Serial.print(",");

  Serial.print("GY:");
  Serial.print(gy_f);

  Serial.print(",");

  Serial.print("GZ:");
  Serial.print(gz_f);

  Serial.print(",");

  //--------------------------------
  // ACCEL
  //--------------------------------

  Serial.print("AX:");
  Serial.print(ax_f);

  Serial.print(",");

  Serial.print("AY:");
  Serial.print(ay_f);

  Serial.print(",");

  Serial.print("AZ:");
  Serial.print(az_f);

  Serial.print(",");

  //--------------------------------
  // DHT22
  //--------------------------------

  Serial.print("TEMP:");
  Serial.print(temperature);

  Serial.print(",");

  Serial.print("HUM:");
  Serial.println(humidity);
}