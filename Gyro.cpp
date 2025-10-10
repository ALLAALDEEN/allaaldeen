#include "Arduino.h"
#include <Wire.h>
#include "Gyro.h"

void Gyro::setupwire()
{
  // Wire initialization
  Wire.begin();
  Wire.beginTransmission(0x68);         // 0x68 is the address for the MPU6050
  Wire.write(0x6B);                     // calling register 'PWR_MGMT_1' 
  Wire.write(0);                        // setting it to zero to activate the module
  Wire.endTransmission(true);           // end

  // Setting gyro configuration to FS_SEL 1 (±500°/s) with 65.5 LSB/°/s
  Wire.beginTransmission(0x68);         // start communicating to MPU6050
  Wire.write(0x1B);                     // 0x1B is the Gyro_Config Register
  Wire.write(0x08);                     // FS_SEL = 1 (±500°/s)
  Wire.endTransmission();               // end

  // Setting accelerometer configuration to AFS_SEL 2 (±8g) with 4096 LSB/g
  Wire.beginTransmission(0x68);         // start communicating to MPU6050
  Wire.write(0x1C);                     // 0x1C is the Acc_Config Register
  Wire.write(0x10);                     // AFS_SEL = 2 (±8g)
  Wire.endTransmission();               // end

  delay(100);
}

void Gyro::SetupWire(double TIME)
{
  // Initialize time
  countTime = false;
  Time = TIME;
  setupwire();
  calibrateGyro();
}

void Gyro::SetupWire()
{
  countTime = true;
  setupwire();
  calibrateGyro();
}

void Gyro::calculateError()
{
  if (countTime)    // if 'time per iteration' is not set, count time between iterations
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
  // Set Up Wire Communication
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);  // request a total of 14 registers

  // Reading Data
  RawAcc.x = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  RawAcc.y = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  RawAcc.z = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) - temperature

  RawGyro.x = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)    
  RawGyro.y = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  RawGyro.z = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void Gyro::calculateAngle()
{
  // Scaling values with improved precision
  GyroScaled.x = ((float)(RawGyro.x - GyroCal.x)) / ScaleGyro;
  GyroScaled.y = ((float)(RawGyro.y - GyroCal.y)) / ScaleGyro;

  // Z Lowpass filter for Z axis - improved dead zone
  if (abs(RawGyro.z - GyroCal.z) < limZ)
    GyroScaled.z = 0;
  else
    GyroScaled.z = ((float)(RawGyro.z - GyroCal.z)) / ScaleGyro;

  // Integrating angular speed over time
  Gyro_angle.x += GyroScaled.x * Time;
  Gyro_angle.y += GyroScaled.y * Time;
  Gyro_angle.z += GyroScaled.z * Time;

  // Involving opposite component multiplied by sin of z - improved coupling
  float sin_z = sin(GyroScaled.z * Time * deg_to_rad);
  Gyro_angle.x += Gyro_angle.y * sin_z;
  Gyro_angle.y -= Gyro_angle.x * sin_z;

  // Calculating Acceleration Angle and total Acc vector
  Acc_totalVec = sqrt(pow(RawAcc.x, 2) + pow(RawAcc.y, 2) + pow(RawAcc.z, 2)) / ScaleAcc;

  // Improved accelerometer angle calculation with better Z-axis handling
  float acc_z_corrected = RawAcc.z * 0.85;  // Reduce Z influence for better stability
  Acc_angle.x = atan2(RawAcc.y, sqrt(pow(RawAcc.x, 2) + pow(acc_z_corrected, 2))) * rad_to_deg;
  Acc_angle.y = -atan2(RawAcc.x, sqrt(pow(RawAcc.y, 2) + pow(acc_z_corrected, 2))) * rad_to_deg;

  // Set acc_angle absolute on first iteration or calculate angle based on complementary filter
  if (GyroSet)
  {
    Gyro_angle.x = Acc_angle.x;
    Gyro_angle.y = Acc_angle.y;
    Gyro_angle.z = 0;
    GyroSet = false;
  }

  // Checking if Acceleration-Angle is valid - improved validation
  if (RawAcc.z > -100 && Acc_totalVec > 0.1 && Acc_totalVec < 2.0)
  {
    // Complementary filter with improved coefficients
    Gyro_angle.x = 0.995 * Gyro_angle.x + 0.005 * Acc_angle.x;
    Gyro_angle.y = 0.995 * Gyro_angle.y + 0.005 * Acc_angle.y;
  }

  // Calculating Error, based on Calibration and Target Angle
  error.x = Gyro_angle.x - target.x - cal.x;
  error.y = Gyro_angle.y - target.y - cal.y;
  error.z = Gyro_angle.z - target.z - cal.z;
}

void Gyro::calibrateGyro()
{
  double x = 0;
  double y = 0;
  double z = 0;

  int n = 2000;  // Increased calibration samples for better accuracy

  for (int i = 0; i < n; i++)
  {
    readingMPU();

    x += RawGyro.x;
    y += RawGyro.y;
    z += RawGyro.z;
    
    delay(1);  // Small delay for stable readings
  }

  GyroCal.x = x / n;
  GyroCal.y = y / n;
  GyroCal.z = z / n;

  Serial.println("Gyro calibrated");
  delay(100);
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

    delay(2);  // Small delay for stable readings
  }

  temp.x = tempX / n;
  temp.y = tempY / n;
  temp.z = 0;  // Z calibration not needed

  return temp;
}

void Gyro::zeroYaw(bool lt)
{
  if (lt)
    Gyro_angle.z = 0;
}