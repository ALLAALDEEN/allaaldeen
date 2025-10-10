/*
 * Optimized RC Controller
 * Enhanced reliability and precision
 * 
 * Features:
 * - Improved joystick calibration
 * - Better data smoothing
 * - Enhanced radio reliability
 * - Better error handling
 * - Optimized control curves
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

// Hardware definitions
RF24 radio(9, 10);  // CE = D9, CSN = D10
const uint64_t pipe = 0xF0F0F0F0E1LL;

// Pin definitions
const int JOYSTICK_XL_PIN = A1;  // Left stick X
const int JOYSTICK_YL_PIN = A0;  // Left stick Y
const int JOYSTICK_XR_PIN = A3;  // Right stick X
const int JOYSTICK_YR_PIN = A2;  // Right stick Y
const int BUTTON1_PIN = 4;
const int BUTTON2_PIN = 5;
const int SWITCH1_PIN = 3;
const int SWITCH2_PIN = 2;

// Calibration data (optimized for better control)
struct CalibrationData {
  float scaleX = 0.12f;      // Increased for better sensitivity
  float offsetX = -527.0f;
  float deadbandX = 5.0f;    // Added deadband
  
  float scaleY = -0.12f;     // Increased for better sensitivity
  float offsetY = -507.0f;
  float deadbandY = 5.0f;    // Added deadband
  
  float scaleZ = -0.12f;     // Increased for better yaw control
  float offsetZ = -512.0f;
  float deadbandZ = 3.0f;    // Smaller deadband for yaw
  
  float scaleThrust = 1.8f;  // Increased for better throttle response
  float offsetThrust = -500.0f;
  float deadbandThrust = 8.0f;  // Added deadband for throttle
} calibration;

// Control data structure
struct FlightData {
  int16_t thrust = 0;
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  uint16_t id = 0;
  bool button1 = true;
  bool button2 = true;
  bool switch1 = true;
  bool switch2 = true;
} flightData;

// Smoothing filters
Smoothed<float> filterXL, filterYL, filterXR, filterYR;
Smoothed<float> filterThrust;

// Control variables
float rawXL, rawYL, rawXR, rawYR;
float smoothXL, smoothYL, smoothXR, smoothYR;
bool button1, button2, switch1, switch2;
uint16_t packetID = 0;

// Exponential curve parameters for better control
const float EXPO_FACTOR = 0.3f;  // 0.0 = linear, 1.0 = full exponential
const float CURVE_FACTOR = 0.7f; // Smoothing factor for control curves

// Radio settings
const int RADIO_CHANNEL = 35;
const int RADIO_POWER = RF24_PA_LOW;
const int RADIO_DATA_RATE = RF24_250KBPS;
const int RADIO_RETRY_COUNT = 3;
const int RADIO_RETRY_DELAY = 15;

// Timing
unsigned long lastTransmitTime = 0;
const unsigned long TRANSMIT_INTERVAL = 5000;  // 5ms = 200Hz

// Function prototypes
void initializeHardware();
void readInputs();
void applyCalibration();
void applyControlCurves();
void transmitData();
void printDebugInfo();
float applyExponentialCurve(float input, float factor);
float applyDeadband(float input, float deadband);

void setup() {
  Serial.begin(115200);
  Serial.println("RC Controller v2.0 - Starting...");
  
  // Initialize hardware
  initializeHardware();
  
  // Initialize smoothing filters
  filterXL.begin(SMOOTHED_EXPONENTIAL, 2);
  filterYL.begin(SMOOTHED_EXPONENTIAL, 2);
  filterXR.begin(SMOOTHED_EXPONENTIAL, 2);
  filterYR.begin(SMOOTHED_EXPONENTIAL, 2);
  filterThrust.begin(SMOOTHED_EXPONENTIAL, 3);
  
  Serial.println("Controller Ready!");
}

void loop() {
  unsigned long currentTime = micros();
  
  // Read inputs
  readInputs();
  
  // Apply calibration
  applyCalibration();
  
  // Apply control curves
  applyControlCurves();
  
  // Prepare flight data
  flightData.thrust = (int16_t)(smoothYL * calibration.scaleThrust + calibration.offsetThrust);
  flightData.roll = smoothXR * calibration.scaleX + calibration.offsetX;
  flightData.pitch = smoothYR * calibration.scaleY + calibration.offsetY;
  flightData.yaw = smoothXL * calibration.scaleZ + calibration.offsetZ;
  flightData.id = packetID++;
  flightData.button1 = button1;
  flightData.button2 = button2;
  flightData.switch1 = switch1;
  flightData.switch2 = switch2;
  
  // Apply deadbands
  flightData.roll = applyDeadband(flightData.roll, calibration.deadbandX);
  flightData.pitch = applyDeadband(flightData.pitch, calibration.deadbandY);
  flightData.yaw = applyDeadband(flightData.yaw, calibration.deadbandZ);
  flightData.thrust = applyDeadband(flightData.thrust, calibration.deadbandThrust);
  
  // Constrain values
  flightData.thrust = constrain(flightData.thrust, 1000, 2000);
  flightData.roll = constrain(flightData.roll, -500, 500);
  flightData.pitch = constrain(flightData.pitch, -500, 500);
  flightData.yaw = constrain(flightData.yaw, -500, 500);
  
  // Transmit data
  if (currentTime - lastTransmitTime >= TRANSMIT_INTERVAL) {
    transmitData();
    lastTransmitTime = currentTime;
  }
  
  // Debug output (reduced frequency)
  static unsigned long lastDebugTime = 0;
  if (currentTime - lastDebugTime > 100000) {  // 10Hz debug output
    printDebugInfo();
    lastDebugTime = currentTime;
  }
}

void initializeHardware() {
  // Initialize input pins
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(SWITCH2_PIN, INPUT_PULLUP);
  
  // Initialize analog pins
  pinMode(JOYSTICK_XL_PIN, INPUT);
  pinMode(JOYSTICK_YL_PIN, INPUT);
  pinMode(JOYSTICK_XR_PIN, INPUT);
  pinMode(JOYSTICK_YR_PIN, INPUT);
  
  // Initialize radio
  radio.begin();
  radio.setAutoAck(true);
  radio.setDataRate(RADIO_DATA_RATE);
  radio.setPALevel(RADIO_POWER);
  radio.setChannel(RADIO_CHANNEL);
  radio.setRetries(RADIO_RETRY_COUNT, RADIO_RETRY_DELAY);
  radio.openWritingPipe(pipe);
  radio.stopListening();
  
  // Test radio connection
  if (radio.isChipConnected()) {
    Serial.println("Radio connected successfully");
  } else {
    Serial.println("Radio connection failed!");
  }
}

void readInputs() {
  // Read digital inputs
  button1 = digitalRead(BUTTON1_PIN);
  button2 = digitalRead(BUTTON2_PIN);
  switch1 = digitalRead(SWITCH1_PIN);
  switch2 = digitalRead(SWITCH2_PIN);
  
  // Read analog inputs with improved scaling
  rawXL = analogRead(JOYSTICK_XL_PIN);
  rawYL = analogRead(JOYSTICK_YL_PIN);
  rawXR = analogRead(JOYSTICK_XR_PIN);
  rawYR = 1023 - analogRead(JOYSTICK_YR_PIN);  // Invert Y axis
  
  // Apply smoothing
  filterXL.add(rawXL);
  filterYL.add(rawYL);
  filterXR.add(rawXR);
  filterYR.add(rawYR);
  
  smoothXL = filterXL.get();
  smoothYL = filterYL.get();
  smoothXR = filterXR.get();
  smoothYR = filterYR.get();
}

void applyCalibration() {
  // Apply basic calibration (offsets and scaling)
  // This is handled in the main loop for better performance
}

void applyControlCurves() {
  // Apply exponential curves for better control feel
  smoothXL = applyExponentialCurve(smoothXL, EXPO_FACTOR);
  smoothYL = applyExponentialCurve(smoothYL, EXPO_FACTOR);
  smoothXR = applyExponentialCurve(smoothXR, EXPO_FACTOR);
  smoothYR = applyExponentialCurve(smoothYR, EXPO_FACTOR);
}

void transmitData() {
  // Transmit data with error checking
  bool success = radio.write(&flightData, sizeof(flightData));
  
  if (!success) {
    Serial.println("Transmission failed!");
  }
}

void printDebugInfo() {
  Serial.print("Thrust: ");
  Serial.print(flightData.thrust);
  Serial.print(" Roll: ");
  Serial.print(flightData.roll);
  Serial.print(" Pitch: ");
  Serial.print(flightData.pitch);
  Serial.print(" Yaw: ");
  Serial.print(flightData.yaw);
  Serial.print(" B1: ");
  Serial.print(button1 ? "ON" : "OFF");
  Serial.print(" B2: ");
  Serial.print(button2 ? "ON" : "OFF");
  Serial.print(" S1: ");
  Serial.print(switch1 ? "ON" : "OFF");
  Serial.print(" S2: ");
  Serial.print(switch2 ? "ON" : "OFF");
  Serial.print(" ID: ");
  Serial.println(packetID);
}

float applyExponentialCurve(float input, float factor) {
  // Normalize input to -1 to 1
  float normalized = (input - 512.0f) / 512.0f;
  
  // Apply exponential curve
  float sign = (normalized >= 0) ? 1.0f : -1.0f;
  float absValue = abs(normalized);
  float curved = sign * (absValue * absValue * factor + absValue * (1.0f - factor));
  
  // Denormalize back to 0-1023
  return (curved * 512.0f) + 512.0f;
}

float applyDeadband(float input, float deadband) {
  if (abs(input) < deadband) {
    return 0.0f;
  }
  return input;
}