/*
 * Optimized Barometer Altitude Control
 * Enhanced precision and stability
 * 
 * Features:
 * - Improved pressure filtering
 * - Better altitude hold
 * - Enhanced PID tuning
 * - Adaptive control
 */

#include "MS5611.h"
#include <Smoothed.h>

// Barometer instance
MS5611 barometer(0x77);

// Pressure filtering
Smoothed<float> pressureFilter;
Smoothed<float> altitudeFilter;

// Altitude control parameters
struct AltitudeControl {
  float kp = 25.0f;        // Increased for better response
  float ki = 5.0f;         // Increased for steady-state
  float kd = 15.0f;        // Increased for damping
  float maxOutput = 600.0f; // Increased max output
  float minOutput = -600.0f;
  
  float integral = 0.0f;
  float previousError = 0.0f;
  float output = 0.0f;
  
  float setpoint = 0.0f;
  float input = 0.0f;
  bool active = false;
  bool initialized = false;
} altControl;

// Pressure data
struct PressureData {
  float rawPressure = 0.0f;
  float filteredPressure = 0.0f;
  float altitude = 0.0f;
  float groundPressure = 101325.0f;  // Sea level pressure in Pa
  float pressureChange = 0.0f;
  
  // Pressure change buffer for derivative calculation
  float pressureBuffer[20];
  int bufferIndex = 0;
  float pressureSum = 0.0f;
} pressureData;

// Timing
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 5000;  // 5ms = 200Hz

// Constants
const float SEA_LEVEL_PRESSURE = 101325.0f;  // Pa
const float PRESSURE_SCALE = 0.01f;          // Scale factor for pressure
const float ALTITUDE_SCALE = 8.3f;           // Meters per hPa

// Function prototypes
void initializeBarometer();
void updatePressure();
void calculateAltitude();
void updateAltitudeControl(float targetThrust, float currentThrust);
float getAltitudeOutput();
void resetAltitudeControl();
void calibrateGroundPressure();

void initializeBarometer() {
  // Initialize barometer
  if (barometer.begin()) {
    barometer.setOversampling(OSR_ULTRA_HIGH);  // Best accuracy
    Serial.println("Barometer initialized successfully");
  } else {
    Serial.println("Barometer initialization failed!");
    return;
  }
  
  // Initialize filters
  pressureFilter.begin(SMOOTHED_AVERAGE, 20);  // Increased for better smoothing
  altitudeFilter.begin(SMOOTHED_AVERAGE, 15);
  
  // Calibrate ground pressure
  calibrateGroundPressure();
  
  // Initialize altitude control
  altControl.setpoint = pressureData.groundPressure;
  altControl.initialized = true;
  
  Serial.println("Barometer system ready");
}

void updatePressure() {
  unsigned long currentTime = micros();
  
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    // Read pressure from barometer
    if (barometer.read()) {
      pressureData.rawPressure = barometer.getPressure();
      
      // Apply filtering
      pressureFilter.add(pressureData.rawPressure);
      pressureData.filteredPressure = pressureFilter.get();
      
      // Calculate altitude
      calculateAltitude();
      
      // Update pressure change buffer
      updatePressureBuffer();
      
      lastUpdateTime = currentTime;
    }
  }
}

void calculateAltitude() {
  // Calculate altitude using barometric formula
  float pressureRatio = pressureData.filteredPressure / pressureData.groundPressure;
  pressureData.altitude = 44330.0f * (1.0f - pow(pressureRatio, 0.1903f));
  
  // Apply altitude filtering
  altitudeFilter.add(pressureData.altitude);
  pressureData.altitude = altitudeFilter.get();
}

void updatePressureBuffer() {
  // Remove old value
  pressureData.pressureSum -= pressureData.pressureBuffer[pressureData.bufferIndex];
  
  // Add new value
  pressureData.pressureBuffer[pressureData.bufferIndex] = pressureData.filteredPressure;
  pressureData.pressureSum += pressureData.filteredPressure;
  
  // Update index
  pressureData.bufferIndex = (pressureData.bufferIndex + 1) % 20;
  
  // Calculate pressure change rate
  float averagePressure = pressureData.pressureSum / 20.0f;
  pressureData.pressureChange = pressureData.filteredPressure - averagePressure;
}

void updateAltitudeControl(float targetThrust, float currentThrust) {
  if (!altControl.initialized) return;
  
  // Check if altitude hold should be active
  if (targetThrust > 1400 && targetThrust < 1600) {
    if (!altControl.active) {
      // Initialize altitude hold
      altControl.setpoint = pressureData.filteredPressure;
      altControl.active = true;
      altControl.integral = 0.0f;
      altControl.previousError = 0.0f;
    }
    
    // Manual altitude adjustment
    if (targetThrust > 1500) {
      altControl.setpoint = pressureData.filteredPressure;
      altControl.integral = 0.0f;  // Reset integral on manual change
    } else if (targetThrust < 1400) {
      altControl.setpoint = pressureData.filteredPressure;
      altControl.integral = 0.0f;  // Reset integral on manual change
    }
    
    // Calculate PID output
    altControl.input = pressureData.filteredPressure;
    float error = altControl.input - altControl.setpoint;
    
    // Proportional term
    float pTerm = altControl.kp * error;
    
    // Integral term
    altControl.integral += error * 0.005f;  // 200Hz
    altControl.integral = constrain(altControl.integral, 
                                   altControl.minOutput / altControl.ki, 
                                   altControl.maxOutput / altControl.ki);
    float iTerm = altControl.ki * altControl.integral;
    
    // Derivative term (using pressure change rate)
    float dTerm = altControl.kd * pressureData.pressureChange;
    
    // Calculate output
    altControl.output = pTerm + iTerm + dTerm;
    altControl.output = constrain(altControl.output, 
                                 altControl.minOutput, 
                                 altControl.maxOutput);
    
    // Update previous error
    altControl.previousError = error;
    
  } else {
    // Deactivate altitude hold
    altControl.active = false;
    altControl.output = 0.0f;
    altControl.integral = 0.0f;
  }
}

float getAltitudeOutput() {
  return altControl.output;
}

void resetAltitudeControl() {
  altControl.active = false;
  altControl.output = 0.0f;
  altControl.integral = 0.0f;
  altControl.previousError = 0.0f;
  altControl.setpoint = pressureData.filteredPressure;
}

void calibrateGroundPressure() {
  Serial.println("Calibrating ground pressure...");
  
  // Take multiple readings for calibration
  float pressureSum = 0.0f;
  int readings = 100;
  
  for (int i = 0; i < readings; i++) {
    if (barometer.read()) {
      pressureSum += barometer.getPressure();
    }
    delay(10);
  }
  
  pressureData.groundPressure = pressureSum / readings;
  
  Serial.print("Ground pressure calibrated: ");
  Serial.print(pressureData.groundPressure);
  Serial.println(" Pa");
}

// Additional utility functions
float getCurrentAltitude() {
  return pressureData.altitude;
}

float getCurrentPressure() {
  return pressureData.filteredPressure;
}

bool isAltitudeHoldActive() {
  return altControl.active;
}

void setAltitudeSetpoint(float altitude) {
  // Convert altitude to pressure setpoint
  float pressureRatio = pow(1.0f - (altitude / 44330.0f), 5.255f);
  altControl.setpoint = pressureData.groundPressure * pressureRatio;
}

void adjustAltitudeGains(float kp, float ki, float kd) {
  altControl.kp = kp;
  altControl.ki = ki;
  altControl.kd = kd;
}