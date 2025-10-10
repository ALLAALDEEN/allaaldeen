#include "Arduino.h"
#include <Wire.h>
#include "Gyro.h"

// FIX: Consistent function naming (was setupwire, now setupWire)
void Gyro::setupWire()
{
  // Initialize I2C communication
  Wire.begin();
  Wire.beginTransmission(0x68);  // MPU6050 address
  Wire.write(0x6B);              // PWR_MGMT_1 register
  Wire.write(0);                 // Wake up MPU6050
  Wire.endTransmission(true);

  // Configure gyroscope: FS_SEL=1 → ±500°/s (65.5 LSB/°/s)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);              // GYRO_CONFIG register
  Wire.write(0x08);              // FS_SEL=1
  Wire.endTransmission();

  // Configure accelerometer: AFS_SEL=2 → ±8g (4096 LSB/g)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);              // ACCEL_CONFIG register
  Wire.write(0x10);              // AFS_SEL=2
  Wire.endTransmission();

  delay(100);
}

void Gyro::SetupWire(double TIME)
{
  countTime = false;
  Time = TIME;
  setupWire();
  calibrateGyro();
}

void Gyro::SetupWire()
{
  countTime = true;
  setupWire();
  calibrateGyro();
}

void Gyro::calculateError()
{
  if (countTime)  // If time per iteration not set, measure it
  {
    Time = micros() - prevTime;
    Time *= micro_to_sec;
    prevTime = micros();
  }

  readingMPU();
  calculateAngle();
}

void Gyro::setTarget(Vec3 Target)
{
  target = Target;
}

void Gyro::setCalibration(Vec3 Cal)
{
  cal = Cal;
}

void Gyro::readingMPU()
{
  // Read all sensor data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // Starting register (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);  // Request 14 registers

  // Read accelerometer data
  RawAcc.x = Wire.read() << 8 | Wire.read();  // ACCEL_XOUT
  RawAcc.y = Wire.read() << 8 | Wire.read();  // ACCEL_YOUT
  RawAcc.z = Wire.read() << 8 | Wire.read();  // ACCEL_ZOUT

  tmp = Wire.read() << 8 | Wire.read();       // TEMP_OUT (unused)

  // Read gyroscope data
  RawGyro.x = Wire.read() << 8 | Wire.read(); // GYRO_XOUT
  RawGyro.y = Wire.read() << 8 | Wire.read(); // GYRO_YOUT
  RawGyro.z = Wire.read() << 8 | Wire.read(); // GYRO_ZOUT
}

void Gyro::calculateAngle()
{
  // Scale and calibrate gyro values
  GyroScaled.x = (RawGyro.x - GyroCal.x) / ScaleGyro;
  GyroScaled.y = (RawGyro.y - GyroCal.y) / ScaleGyro;

  // Z-axis low-pass filter (deadband)
  if (abs(RawGyro.z - GyroCal.z) < limZ)
    GyroScaled.z = 0;
  else
    GyroScaled.z = (RawGyro.z - GyroCal.z) / ScaleGyro;

  // Integrate angular velocity to get angle
  Gyro_angle.x += GyroScaled.x * Time;
  Gyro_angle.y += GyroScaled.y * Time;
  Gyro_angle.z += GyroScaled.z * Time;

  // Yaw drift compensation
  Gyro_angle.x += Gyro_angle.y * sin(GyroScaled.z * Time * deg_to_rad);
  Gyro_angle.y -= Gyro_angle.x * sin(GyroScaled.z * Time * deg_to_rad);

  // Calculate acceleration angles
  Acc_totalVec = sqrt(pow(RawAcc.x, 2) + pow(RawAcc.y, 2) + pow(RawAcc.z, 2)) / ScaleAcc;

  // FIX: Improved accelerometer angle calculation with safety checks
  float acc_sqrt_y = sqrt(pow(RawAcc.x, 2) + pow(RawAcc.z * 0.85, 2));
  float acc_sqrt_x = sqrt(pow(RawAcc.y, 2) + pow(RawAcc.z * 0.85, 2));

  // Avoid division by zero
  if (acc_sqrt_y > 1.0) {
    Acc_angle.x = atan(RawAcc.y / acc_sqrt_y) * rad_to_deg;
  }
  if (acc_sqrt_x > 1.0) {
    Acc_angle.y = -atan(RawAcc.x / acc_sqrt_x) * rad_to_deg;
  }

  // Initialize on first iteration
  if (GyroSet)
  {
    Gyro_angle.x = Acc_angle.x;
    Gyro_angle.y = Acc_angle.y;
    Gyro_angle.z = 0;
    GyroSet = false;
  }

  // Complementary filter: merge gyro and accelerometer data
  // Only trust accelerometer when total acceleration is near 1g (not in freefall or high acceleration)
  if (RawAcc.z > -100 && Acc_totalVec > 0.1 && Acc_totalVec < 2.0)
  {
    Gyro_angle.x = 0.99 * Gyro_angle.x + 0.01 * Acc_angle.x;
    Gyro_angle.y = 0.99 * Gyro_angle.y + 0.01 * Acc_angle.y;
  }

  // Calculate error (difference between actual and target angle)
  error.x = Gyro_angle.x - target.x - cal.x;
  error.y = Gyro_angle.y - target.y - cal.y;
  error.z = Gyro_angle.z - target.z - cal.z;
}

void Gyro::calibrateGyro()
{
  double x = 0;
  double y = 0;
  double z = 0;

  int n = 1500;

  for (int i = 0; i < n; i++)
  {
    readingMPU();
    x += RawGyro.x;
    y += RawGyro.y;
    z += RawGyro.z;
    delay(1);  // FIX: Small delay for stable readings
  }

  delay(100);

  GyroCal.x = x / n;
  GyroCal.y = y / n;
  GyroCal.z = z / n;
}

Vec3 Gyro::calibrate(int n)
{
  float tempX = 0;
  float tempY = 0;
  Vec3 temp;

  setTarget({0, 0, 0});
  setCalibration({0, 0, 0});
  calibrateGyro();

  for (int i = 0; i < n; i++)
  {
    calculateError();
    tempX += error.x;
    tempY += error.y;
    delay(5);  // FIX: Stable sampling
  }

  temp.x = tempX / n;
  temp.y = tempY / n;
  temp.z = 0;

  return temp;
}

void Gyro::zeroYaw(bool reset)
{
  if (reset)
    Gyro_angle.z = 0;
}
