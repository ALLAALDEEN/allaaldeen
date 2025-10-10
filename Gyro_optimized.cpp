/*
 * Optimized Gyroscope Library Implementation
 * This file provides backward compatibility with the original Gyro class
 */

#include "Gyro_optimized.h"

// Backward compatibility implementation
void Gyro::SetupWire(double TIME) {
  initialize(TIME);
}

void Gyro::SetupWire() {
  initialize();
}

void Gyro::calculateError() {
  calculateError();
}

void Gyro::setTarget(Vec3 Target) {
  setTarget(Target);
}

void Gyro::setCalibration(Vec3 Cal) {
  setCalibration(Cal);
}

Vec3 Gyro::calibrate(int n) {
  return calibrate(n);
}

void Gyro::zeroYaw(bool lt) {
  zeroYaw(lt);
}

void Gyro::setupwire() {
  setupMPU6050();
}

void Gyro::readingMPU() {
  readMPU6050();
}

void Gyro::calculateAngle() {
  processData();
  calculateAngles();
}

void Gyro::calibrateGyro() {
  calibrate(1500);
}

// Additional utility functions
void Gyro::setupMPU6050() {
  Wire.begin();
  
  // Wake up MPU6050
  writeRegister(0x6B, 0x00);
  delay(100);
  
  // Configure gyroscope (±500deg/s)
  writeRegister(0x1B, 0x08);
  
  // Configure accelerometer (±2g)
  writeRegister(0x1C, 0x00);
  
  // Configure DLPF (44Hz)
  writeRegister(0x1A, 0x03);
  
  delay(100);
}

void Gyro::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void Gyro::readMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  
  // Read accelerometer data
  rawAccel.x = (Wire.read() << 8 | Wire.read());
  rawAccel.y = (Wire.read() << 8 | Wire.read());
  rawAccel.z = (Wire.read() << 8 | Wire.read());
  
  // Read temperature
  temperature = (Wire.read() << 8 | Wire.read()) / 340.0f + 36.53f;
  
  // Read gyroscope data
  rawGyro.x = (Wire.read() << 8 | Wire.read());
  rawGyro.y = (Wire.read() << 8 | Wire.read());
  rawGyro.z = (Wire.read() << 8 | Wire.read());
}

void Gyro::processData() {
  // Scale accelerometer data
  Vec3 accelScaled = rawAccel * (1.0f / ACCEL_SCALE);
  
  // Apply low-pass filter to accelerometer
  accelFiltered = accelFiltered * ACCEL_LOWPASS_ALPHA + 
                 accelScaled * (1.0f - ACCEL_LOWPASS_ALPHA);
  
  // Scale gyroscope data and apply calibration
  gyroScaled = (rawGyro - gyroCalibration) * (1.0f / GYRO_SCALE);
  
  // Apply deadband to gyro Z-axis
  if (abs(gyroScaled.z) < GYRO_DEADBAND) {
    gyroScaled.z = 0.0f;
  }
  
  // Apply high-pass filter to gyroscope
  gyroFiltered = gyroFiltered * GYRO_HIGHPASS_ALPHA + 
                gyroScaled * (1.0f - GYRO_HIGHPASS_ALPHA);
}

void Gyro::calculateAngles() {
  // Calculate accelerometer angles
  accelMagnitude = sqrt(accelFiltered.x * accelFiltered.x + 
                       accelFiltered.y * accelFiltered.y + 
                       accelFiltered.z * accelFiltered.z);
  
  if (accelMagnitude > 0.1f) {  // Valid acceleration data
    accelAngle.x = atan2(accelFiltered.y, 
                        sqrt(accelFiltered.x * accelFiltered.x + 
                             accelFiltered.z * accelFiltered.z)) * RAD_TO_DEG;
    accelAngle.y = atan2(-accelFiltered.x, 
                        sqrt(accelFiltered.y * accelFiltered.y + 
                             accelFiltered.z * accelFiltered.z)) * RAD_TO_DEG;
  }
  
  // Integrate gyroscope data
  gyroAngle += gyroFiltered * deltaTime;
  
  // Apply complementary filter
  if (firstRun) {
    gyroAngle.x = accelAngle.x;
    gyroAngle.y = accelAngle.y;
    gyroAngle.z = 0.0f;
    firstRun = false;
  } else {
    gyroAngle.x = complementaryAlpha * gyroAngle.x + 
                 (1.0f - complementaryAlpha) * accelAngle.x;
    gyroAngle.y = complementaryAlpha * gyroAngle.y + 
                 (1.0f - complementaryAlpha) * accelAngle.y;
  }
  
  // Apply tilt compensation for yaw
  gyroAngle.x += gyroAngle.y * sin(gyroAngle.z * DEG_TO_RAD);
  gyroAngle.y -= gyroAngle.x * sin(gyroAngle.z * DEG_TO_RAD);
}

void Gyro::calculateError() {
  error.x = gyroAngle.x - targetAngle.x - calibration.x;
  error.y = gyroAngle.y - targetAngle.y - calibration.y;
  error.z = gyroAngle.z - targetAngle.z - calibration.z;
  
  // Normalize yaw error to -180 to +180
  while (error.z > 180.0f) error.z -= 360.0f;
  while (error.z < -180.0f) error.z += 360.0f;
}