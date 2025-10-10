/*
 * Optimized Gyroscope Library
 * Enhanced sensor fusion with complementary filter
 * Improved calibration and error handling
 */

#ifndef GYRO_OPTIMIZED_H
#define GYRO_OPTIMIZED_H

#include "Arduino.h"
#include <Wire.h>

// Vector structure for 3D data
struct Vec3 {
  float x, y, z;
  
  Vec3() : x(0), y(0), z(0) {}
  Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
  
  Vec3 operator+(const Vec3& other) const {
    return Vec3(x + other.x, y + other.y, z + other.z);
  }
  
  Vec3 operator-(const Vec3& other) const {
    return Vec3(x - other.x, y - other.y, z - other.z);
  }
  
  Vec3 operator*(float scalar) const {
    return Vec3(x * scalar, y * scalar, z * scalar);
  }
  
  Vec3& operator+=(const Vec3& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }
  
  Vec3& operator-=(const Vec3& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }
};

class GyroOptimized {
private:
  // Raw sensor data
  Vec3 rawAccel;
  Vec3 rawGyro;
  float temperature = 0.0f;
  
  // Processed data
  Vec3 gyroScaled;
  Vec3 gyroAngle;
  Vec3 accelAngle;
  Vec3 targetAngle;
  Vec3 calibration;
  Vec3 gyroCalibration;
  
  // Complementary filter variables
  float complementaryAlpha = 0.98f;  // Gyro weight (0-1)
  float accelMagnitude = 0.0f;
  
  // Timing
  float deltaTime = 0.0f;
  unsigned long previousTime = 0;
  bool useExternalTiming = false;
  
  // Calibration
  bool isCalibrated = false;
  bool firstRun = true;
  
  // Scaling factors (MPU6050 specific)
  const float ACCEL_SCALE = 4096.0f;    // LSB/g for ±2g range
  const float GYRO_SCALE = 65.5f;       // LSB/deg/s for ±500deg/s range
  const float GYRO_DEADBAND = 0.5f;     // Deadband for gyro Z-axis
  
  // Mathematical constants
  const float DEG_TO_RAD = PI / 180.0f;
  const float RAD_TO_DEG = 180.0f / PI;
  const float GRAVITY = 9.81f;
  
  // Low-pass filter for accelerometer
  const float ACCEL_LOWPASS_ALPHA = 0.8f;
  Vec3 accelFiltered;
  
  // High-pass filter for gyro
  const float GYRO_HIGHPASS_ALPHA = 0.95f;
  Vec3 gyroFiltered;
  
public:
  Vec3 error;  // Public error vector
  
  // Constructor
  GyroOptimized() {
    // Initialize all vectors to zero
    rawAccel = Vec3();
    rawGyro = Vec3();
    gyroScaled = Vec3();
    gyroAngle = Vec3();
    accelAngle = Vec3();
    targetAngle = Vec3();
    calibration = Vec3();
    gyroCalibration = Vec3();
    error = Vec3();
    accelFiltered = Vec3();
    gyroFiltered = Vec3();
  }
  
  // Initialize with external timing
  void initialize(float controlPeriod) {
    deltaTime = controlPeriod;
    useExternalTiming = true;
    setupMPU6050();
    calibrateGyro();
    previousTime = micros();
  }
  
  // Initialize with internal timing
  void initialize() {
    useExternalTiming = false;
    setupMPU6050();
    calibrateGyro();
    previousTime = micros();
  }
  
  // Main calculation function
  void calculateError() {
    if (!useExternalTiming) {
      unsigned long currentTime = micros();
      deltaTime = (currentTime - previousTime) * 1e-6f;
      previousTime = currentTime;
    }
    
    readMPU6050();
    processData();
    calculateAngles();
    calculateError();
  }
  
  // Set target angles
  void setTarget(const Vec3& target) {
    targetAngle = target;
  }
  
  // Set calibration values
  void setCalibration(const Vec3& cal) {
    calibration = cal;
  }
  
  // Calibrate gyroscope
  Vec3 calibrate(int samples = 2000) {
    Vec3 sum(0, 0, 0);
    
    Serial.println("Calibrating gyroscope...");
    
    for (int i = 0; i < samples; i++) {
      readMPU6050();
      sum += rawGyro;
      
      if (i % 200 == 0) {
        Serial.print("Calibration progress: ");
        Serial.print((i * 100) / samples);
        Serial.println("%");
      }
      
      delay(1);
    }
    
    gyroCalibration = sum * (1.0f / samples);
    isCalibrated = true;
    
    Serial.println("Gyroscope calibration complete!");
    return gyroCalibration;
  }
  
  // Reset yaw angle
  void zeroYaw(bool reset = true) {
    if (reset) {
      gyroAngle.z = 0.0f;
    }
  }
  
  // Get current angles
  Vec3 getAngles() const {
    return gyroAngle;
  }
  
  // Get error angles
  Vec3 getError() const {
    return error;
  }
  
  // Check if calibrated
  bool isCalibrated() const {
    return isCalibrated;
  }

private:
  // Setup MPU6050
  void setupMPU6050() {
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
  
  // Write to MPU6050 register
  void writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(0x68);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }
  
  // Read from MPU6050
  void readMPU6050() {
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
  
  // Process raw sensor data
  void processData() {
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
  
  // Calculate angles using complementary filter
  void calculateAngles() {
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
  
  // Calculate error angles
  void calculateError() {
    error.x = gyroAngle.x - targetAngle.x - calibration.x;
    error.y = gyroAngle.y - targetAngle.y - calibration.y;
    error.z = gyroAngle.z - targetAngle.z - calibration.z;
    
    // Normalize yaw error to -180 to +180
    while (error.z > 180.0f) error.z -= 360.0f;
    while (error.z < -180.0f) error.z += 360.0f;
  }
};

// Backward compatibility typedef
typedef GyroOptimized Gyro;

#endif // GYRO_OPTIMIZED_H