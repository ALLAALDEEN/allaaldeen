#include "Gyro.h"

#include <Wire.h>
#include <math.h>

void Gyro::setupwire() {
  Wire.begin();
  Wire.beginTransmission(0x68);  //0x68 is the address for the MPU6050
  Wire.write(0x6B);              //calling register 'PWR_MGMT_1'
  Wire.write(0);                 //setting it to zero to activate to module
  Wire.endTransmission(true);

  //setting gyro configuration to FS_SEL 1 500deg/sec and 65.5LSB/deg/sec (datasheet)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);  //0x1B is the Gyro_Config Register
  Wire.write(0x08);  //FS_SEL = 1 (65.5 LSB/deg/s)
  Wire.endTransmission();

  //setting accelerometer configuration to AFS_SEL 0x10 = 4096LSB/g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);  //0x1C is the Acc_Config Register
  Wire.write(0x10);  //AFS_SEL = 0x10 (4096 LSB/g)
  Wire.endTransmission();

  delay(100);
}

void Gyro::SetupWire(double TIME) {
  countTime = false;
  Time = TIME;
  setupwire();
  calibrateGyro();
}

void Gyro::SetupWire() {
  countTime = true;
  setupwire();
  calibrateGyro();
}

void Gyro::calculateError() {
  if (countTime) {
    Time = micros() - prevTime;
    Time *= micro_to_sec;
    prevTime = micros();
  }

  readingMPU();
  calculateAngle();
}

void Gyro::setTarget(Vec3 Target) {
  target = Target;
}

void Gyro::setCalibration(Vec3 Cal) {
  cal = Cal;
}

void Gyro::readingMPU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);  // request a total of 14 registers

  RawAcc.x = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  RawAcc.y = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  RawAcc.z = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  RawGyro.x = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  RawGyro.y = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  RawGyro.z = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void Gyro::calculateAngle() {
  GyroScaled.x = ((RawGyro.x - GyroCal.x) / ScaleGyro);
  GyroScaled.y = ((RawGyro.y - GyroCal.y) / ScaleGyro);

  if ((RawGyro.z - GyroCal.z) < limZ && (RawGyro.z - GyroCal.z) > (-limZ))
    GyroScaled.z = 0;
  else
    GyroScaled.z = ((RawGyro.z - GyroCal.z) / ScaleGyro);

  Gyro_angle.x += GyroScaled.x * Time;
  Gyro_angle.y += GyroScaled.y * Time;
  Gyro_angle.z += GyroScaled.z * Time;

  Gyro_angle.x += Gyro_angle.y * sin(GyroScaled.z * Time * deg_to_rad);
  Gyro_angle.y -= Gyro_angle.x * sin(GyroScaled.z * Time * deg_to_rad);

  Acc_totalVec = sqrt(pow(RawAcc.x, 2) + pow(RawAcc.y, 2) + pow(RawAcc.z, 2)) / ScaleAcc;

  Acc_angle.x = atan(RawAcc.y / sqrt(pow(RawAcc.x, 2) + pow(RawAcc.z * 0.85f, 2))) * rad_to_deg;
  Acc_angle.y = -atan(RawAcc.x / sqrt(pow(RawAcc.y, 2) + pow(RawAcc.z * 0.85f, 2))) * rad_to_deg;

  if (GyroSet) {
    Gyro_angle.x = Acc_angle.x;
    Gyro_angle.y = Acc_angle.y;
    Gyro_angle.z = 0;
    GyroSet = false;
  }

  if (RawAcc.z > -100 && Acc_totalVec > 0.1f) {
    Gyro_angle.x = 0.99f * Gyro_angle.x + Acc_angle.x * 0.01f;
    Gyro_angle.y = 0.99f * Gyro_angle.y + Acc_angle.y * 0.01f;
  }

  error.x = Gyro_angle.x - target.x - cal.x;
  error.y = Gyro_angle.y - target.y - cal.y;
  error.z = Gyro_angle.z - target.z - cal.z;
}

void Gyro::calibrateGyro() {
  double x = 0;
  double y = 0;
  double z = 0;

  const int n = 1500;

  for (int i = 0; i < n; i++) {
    readingMPU();
    x += RawGyro.x;
    y += RawGyro.y;
    z += RawGyro.z;
  }

  delay(100);

  GyroCal.x = x / n;
  GyroCal.y = y / n;
  GyroCal.z = z / n;
}

Vec3 Gyro::calibrate(int n) {
  float tempX = 0;
  float tempY = 0;
  Vec3 temp{};

  setTarget(Vec3{0, 0, 0});
  setCalibration(Vec3{0, 0, 0});
  calibrateGyro();

  for (int i = 0; i < n; i++) {
    calculateError();
    tempX += error.x;
    tempY += error.y;
  }

  temp.x = tempX / n;
  temp.y = tempY / n;
  temp.z = 0;

  return temp;
}

void Gyro::zeroYaw(bool lt) {
  if (lt)
    Gyro_angle.z = 0;
}
