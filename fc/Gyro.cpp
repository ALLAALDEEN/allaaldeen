#include <Arduino.h>
#include <Wire.h>
#include "Gyro.h"

void Gyro::setupwire() {
  Wire.begin();
  // Wake up MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);               // PWR_MGMT_1
  Wire.write(0);                  // set to zero (wakes up MPU6050)
  Wire.endTransmission(true);

  // Gyro config: FS_SEL = 1 -> 500 deg/s (65.5 LSB/deg/s)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);               // GYRO_CONFIG
  Wire.write(0x08);               // FS_SEL = 1
  Wire.endTransmission();

  // Accel config: AFS_SEL = 2 -> +/-8g (4096 LSB/g)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);               // ACCEL_CONFIG
  Wire.write(0x10);               // AFS_SEL = 2
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

void Gyro::setTarget(Vec3 Target) { target = Target; }
void Gyro::setCalibration(Vec3 Cal) { cal = Cal; }

void Gyro::readingMPU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  RawAcc.x = (Wire.read() << 8) | Wire.read();
  RawAcc.y = (Wire.read() << 8) | Wire.read();
  RawAcc.z = (Wire.read() << 8) | Wire.read();

  tmp = (Wire.read() << 8) | Wire.read();

  RawGyro.x = (Wire.read() << 8) | Wire.read();
  RawGyro.y = (Wire.read() << 8) | Wire.read();
  RawGyro.z = (Wire.read() << 8) | Wire.read();
}

void Gyro::calculateAngle() {
  // scaling values (deg/s)
  GyroScaled.x = (RawGyro.x - GyroCal.x) / ScaleGyro;
  GyroScaled.y = (RawGyro.y - GyroCal.y) / ScaleGyro;

  // Z low-pass
  const float rawZ = RawGyro.z - GyroCal.z;
  GyroScaled.z = (fabs(rawZ) < limZ) ? 0.0f : (rawZ / ScaleGyro);

  // integrate angular speed over time
  Gyro_angle.x += GyroScaled.x * Time;
  Gyro_angle.y += GyroScaled.y * Time;
  Gyro_angle.z += GyroScaled.z * Time;

  // involve opposite component multiplied by sin(z)
  Gyro_angle.x += Gyro_angle.y * sinf(GyroScaled.z * Time * deg_to_rad);
  Gyro_angle.y -= Gyro_angle.x * sinf(GyroScaled.z * Time * deg_to_rad);

  // acceleration angles
  Acc_totalVec = sqrtf(RawAcc.x * RawAcc.x + RawAcc.y * RawAcc.y + RawAcc.z * RawAcc.z) / ScaleAcc;
  Acc_angle.x = atanf(RawAcc.y / sqrtf(RawAcc.x * RawAcc.x + (RawAcc.z * 0.85f) * (RawAcc.z * 0.85f))) * rad_to_deg;
  Acc_angle.y = -atanf(RawAcc.x / sqrtf(RawAcc.y * RawAcc.y + (RawAcc.z * 0.85f) * (RawAcc.z * 0.85f))) * rad_to_deg;

  if (GyroSet) {
    Gyro_angle.x = Acc_angle.x;
    Gyro_angle.y = Acc_angle.y;
    Gyro_angle.z = 0;
    GyroSet = false;
  }

  if (RawAcc.z > -100 && Acc_totalVec > 0.1f) {
    Gyro_angle.x = 0.99f * Gyro_angle.x + 0.01f * Acc_angle.x;
    Gyro_angle.y = 0.99f * Gyro_angle.y + 0.01f * Acc_angle.y;
  }

  // error
  error.x = Gyro_angle.x - target.x - cal.x;
  error.y = Gyro_angle.y - target.y - cal.y;
  error.z = Gyro_angle.z - target.z - cal.z;
}

void Gyro::calibrateGyro() {
  double x = 0, y = 0, z = 0;
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
  Vec3 temp{0,0,0};

  setTarget({0,0,0});
  setCalibration({0,0,0});
  calibrateGyro();

  for (int i = 0; i < n; i++) {
    calculateError();
    tempX += error.x;
    tempY += error.y;
  }

  temp.x = tempX / n;
  temp.y = tempY / n;
  return temp;
}

void Gyro::zeroYaw(bool lt) {
  if (lt) Gyro_angle.z = 0;
}
