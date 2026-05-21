#include <Wire.h>

#define MPU6050_ADDR 0x68

//-----------------------------------
// RAW DATA
//-----------------------------------

int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

//-----------------------------------
// DATOS CONVERTIDOS
//-----------------------------------

float ax, ay, az;
float gx, gy, gz;

//-----------------------------------
// FILTRO PASA BAJOS
//-----------------------------------

float ax_f = 0;
float ay_f = 0;
float az_f = 0;

float gx_f = 0;
float gy_f = 0;
float gz_f = 0;

//-----------------------------------
// OFFSETS
//-----------------------------------

float gx_offset = 0;
float gy_offset = 0;
float gz_offset = 0;

//-----------------------------------
// ORIENTACION
//-----------------------------------

float roll = 0;
float pitch = 0;
float yaw = 0;

//-----------------------------------
// TIEMPO
//-----------------------------------

unsigned long prevTime;
float dt;

//-----------------------------------
// FILTROS
//-----------------------------------

// Filtro complementario
float alpha = 0.98;

// Filtro pasa bajos
float lowPass = 0.93;

void setup() {

  Serial.begin(115200);

  // TU CONFIGURACION
  Wire.begin(22, 21);

  // I2C RAPIDO
  Wire.setClock(400000);

  setupMPU();

  delay(1000);

  calibrateGyro();

  prevTime = micros();

  Serial.println("IMU LISTA");
}

void loop() {

  readMPU();

  //-----------------------------------
  // TIEMPO
  //-----------------------------------

  unsigned long currentTime = micros();

  dt = (currentTime - prevTime) / 1000000.0;

  prevTime = currentTime;

  //-----------------------------------
  // CONVERSION ACC
  //-----------------------------------

  ax = AcX / 16384.0;
  ay = AcY / 16384.0;
  az = AcZ / 16384.0;

  //-----------------------------------
  // CONVERSION GYRO
  //-----------------------------------

  gx = (GyX - gx_offset) / 131.0;
  gy = (GyY - gy_offset) / 131.0;
  gz = (GyZ - gz_offset) / 131.0;

  //-----------------------------------
  // LOW PASS FILTER
  //-----------------------------------

  ax_f = lowPass * ax_f + (1 - lowPass) * ax;
  ay_f = lowPass * ay_f + (1 - lowPass) * ay;
  az_f = lowPass * az_f + (1 - lowPass) * az;

  gx_f = lowPass * gx_f + (1 - lowPass) * gx;
  gy_f = lowPass * gy_f + (1 - lowPass) * gy;
  gz_f = lowPass * gz_f + (1 - lowPass) * gz;

  //-----------------------------------
  // ANGULOS DEL ACELEROMETRO
  //-----------------------------------

  float accelRoll =
      atan2(ay_f, az_f) * 180.0 / PI;

  float accelPitch =
      atan2(
        -ax_f,
        sqrt(ay_f * ay_f + az_f * az_f)
      ) * 180.0 / PI;

  //-----------------------------------
  // INTEGRACION GYRO
  //-----------------------------------

  roll += gx_f * dt;
  pitch += gy_f * dt;
  yaw += gz_f * dt;

  //-----------------------------------
  // FILTRO COMPLEMENTARIO
  //-----------------------------------

  roll =
      alpha * roll +
      (1 - alpha) * accelRoll;

  pitch =
      alpha * pitch +
      (1 - alpha) * accelPitch;

  //-----------------------------------
  // SALIDA
  //-----------------------------------

  printIMU();

  delay(5);
}

//-----------------------------------
// CONFIG MPU6050
//-----------------------------------

void setupMPU() {

  // WAKE UP
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // GYRO ±250 deg/s
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // ACC ±2g
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  //-----------------------------------
  // DLPF INTERNO
  //-----------------------------------

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);
}

//-----------------------------------
// LEER MPU
//-----------------------------------

void readMPU() {

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

//-----------------------------------
// CALIBRACION
//-----------------------------------

void calibrateGyro() {

  long x = 0;
  long y = 0;
  long z = 0;

  Serial.println("CALIBRANDO IMU");

  for (int i = 0; i < 3000; i++) {

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU6050_ADDR, 6, true);

    int16_t rx = Wire.read() << 8 | Wire.read();
    int16_t ry = Wire.read() << 8 | Wire.read();
    int16_t rz = Wire.read() << 8 | Wire.read();

    x += rx;
    y += ry;
    z += rz;

    delay(1);
  }

  gx_offset = x / 3000.0;
  gy_offset = y / 3000.0;
  gz_offset = z / 3000.0;

  Serial.println("CALIBRACION COMPLETA");
}

//-----------------------------------
// MOSTRAR DATOS
//-----------------------------------

void printIMU() {

  //-----------------------------------
  // ORIENTACION
  //-----------------------------------

  Serial.print("ROLL:");
  Serial.print(roll);

  Serial.print(",");

  Serial.print("PITCH:");
  Serial.print(pitch);

  Serial.print(",");

  Serial.print("YAW:");
  Serial.print(yaw);

  Serial.print(",");

  //-----------------------------------
  // VELOCIDAD ANGULAR
  //-----------------------------------

  Serial.print("GX:");
  Serial.print(gx_f);

  Serial.print(",");

  Serial.print("GY:");
  Serial.print(gy_f);

  Serial.print(",");

  Serial.print("GZ:");
  Serial.print(gz_f);

  Serial.print(",");

  //-----------------------------------
  // ACELERACION
  //-----------------------------------

  Serial.print("AX:");
  Serial.print(ax_f);

  Serial.print(",");

  Serial.print("AY:");
  Serial.print(ay_f);

  Serial.print(",");

  Serial.print("AZ:");
  Serial.println(az_f);
}