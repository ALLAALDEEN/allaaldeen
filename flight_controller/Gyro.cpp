#include <Arduino.h>
#include <Wire.h>
#include "Gyro.h"

void Gyro::setupwire() {
  Wire.begin();

  Wire.beginTransmission(0x68);           // MPU6050
  Wire.write(0x6B);                        // PWR_MGMT_1
  Wire.write(0);                           // wake up
  Wire.endTransmission(true);

  // Gyro config FS_SEL = 1 -> 500 deg/sec (65.5 LSB/deg/sec)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                        // GYRO_CONFIG
  Wire.write(0x08);
  Wire.endTransmission();

  // Accel config AFS_SEL = 0x10 -> 4096 LSB/g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);                        // ACCEL_CONFIG
  Wire.write(0x10);
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

  RawAcc.x = Wire.read() << 8 | Wire.read();
  RawAcc.y = Wire.read() << 8 | Wire.read();
  RawAcc.z = Wire.read() << 8 | Wire.read();

  tmp = Wire.read() << 8 | Wire.read();

  RawGyro.x = Wire.read() << 8 | Wire.read();
  RawGyro.y = Wire.read() << 8 | Wire.read();
  RawGyro.z = Wire.read() << 8 | Wire.read();
}

void Gyro::calculateAngle() {
  // scale gyro
  GyroScaled.x = ((RawGyro.x - GyroCal.x) / ScaleGyro);
  GyroScaled.y = ((RawGyro.y - GyroCal.y) / ScaleGyro);

  if ((RawGyro.z - GyroCal.z) < limZ && (RawGyro.z - GyroCal.z) > (-limZ))
    GyroScaled.z = 0;
  else
    GyroScaled.z = ((RawGyro.z - GyroCal.z) / ScaleGyro);

  // integrate
  Gyro_angle.x += GyroScaled.x * Time;
  Gyro_angle.y += GyroScaled.y * Time;
  Gyro_angle.z += GyroScaled.z * Time;

  // coupling
  Gyro_angle.x += Gyro_angle.y * sin(GyroScaled.z * Time * deg_to_rad);
  Gyro_angle.y -= Gyro_angle.x * sin(GyroScaled.z * Time * deg_to_rad);

  // accel angle
  Acc_totalVec = sqrt(pow(RawAcc.x, 2) + pow(RawAcc.y, 2) + pow(RawAcc.z, 2)) / ScaleAcc;
  Acc_angle.x =  atan(RawAcc.y / sqrt(pow(RawAcc.x, 2) + pow(RawAcc.z * 0.85, 2))) * rad_to_deg;
  Acc_angle.y = -atan(RawAcc.x / sqrt(pow(RawAcc.y, 2) + pow(RawAcc.z * 0.85, 2))) * rad_to_deg;

  if (GyroSet) {
    Gyro_angle.x = Acc_angle.x;
    Gyro_angle.y = Acc_angle.y;
    Gyro_angle.z = 0;
    GyroSet = false;
  }

  if (RawAcc.z > -100 && Acc_totalVec > 0.1) {
    Gyro_angle.x = 0.99 * Gyro_angle.x + Acc_angle.x * 0.01;
    Gyro_angle.y = 0.99 * Gyro_angle.y + Acc_angle.y * 0.01;
  }

  // error
  error.x = Gyro_angle.x - target.x - cal.x;
  error.y = Gyro_angle.y - target.y - cal.y;
  error.z = Gyro_angle.z - target.z - cal.z;
}

void Gyro::calibrateGyro() {
  double x = 0;
  double y = 0;
  double z = 0;
  int n = 1500;

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
  Vec3 temp;

  setTarget({0, 0, 0});
  setCalibration({0, 0, 0});
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
  if (lt) Gyro_angle.z = 0;
}
