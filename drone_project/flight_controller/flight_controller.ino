/**
 * Flight Controller Firmware for Arduino Nano based quadcopter
 *
 * Hardware summary:
 *  - MCU: Arduino Nano
 *  - IMU: MPU6050 (INT -> D2)
 *  - Barometer: MS5611 (GY-63 breakout)
 *  - Radio: nRF24L01 (CE -> D4, CSN -> D10)
 *  - Motors / ESCs: FL -> D3, FR -> D5, RR -> D6, RL -> D7
 *  - Buzzer: D8
 *  - Status LED: D13 (specification requested D7, but that pin is occupied by the rear-left ESC)
 *
 * External library dependencies (install through Arduino Library Manager where possible):
 *  - RF24 by TMRh20
 *  - Servo (built-in)
 *
 * The MPU6050 and MS5611 drivers in this sketch are lightweight register-level
 * implementations to avoid external dependencies. Replace with your preferred
 * sensor libraries if desired.
 *
 * Author: GPT-5 Codex
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// ----------------------------- Configuration ------------------------------

// Radio configuration
constexpr uint8_t RADIO_CE_PIN = 4;
constexpr uint8_t RADIO_CSN_PIN = 10;
constexpr byte RADIO_ADDRESS[6] = "DRNFC";
constexpr uint8_t RADIO_CHANNEL = 100;       // 2.5 GHz + 100 MHz (2.5 GHz total)
constexpr rf24_datarate_e RADIO_DATARATE = RF24_1MBPS;

// Sensor configuration
constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint8_t MPU_INT_PIN = 2;
constexpr uint8_t MS5611_ADDR = 0x77;

// Motor / actuator pins
constexpr uint8_t MOTOR_FL_PIN = 3;
constexpr uint8_t MOTOR_FR_PIN = 5;
constexpr uint8_t MOTOR_RR_PIN = 6;
constexpr uint8_t MOTOR_RL_PIN = 7;
constexpr uint8_t BUZZER_PIN = 8;
constexpr uint8_t LED_PIN = 13;  // Onboard LED, avoids conflict with RL motor on D7

// ESC calibration / limits (use microseconds)
constexpr uint16_t ESC_MIN_US = 1000;
constexpr uint16_t ESC_ARM_US = 1000;
constexpr uint16_t ESC_MAX_US = 2000;

// Control loop timing
constexpr float LOOP_HZ = 250.0f;
constexpr float LOOP_DT = 1.0f / LOOP_HZ;

// PID tuning defaults (adjust through tuning flights)
struct PIDGains {
  float kp;
  float ki;
  float kd;
};

PIDGains pidRoll{4.0f, 0.02f, 15.0f};
PIDGains pidPitch{4.0f, 0.02f, 15.0f};
PIDGains pidYaw{2.0f, 0.00f, 2.5f};

constexpr float INTEGRAL_SATURATION = 200.0f;

// Complementary filter coefficient (0-1); higher favors gyro integration
constexpr float COMPLEMENTARY_ALPHA = 0.98f;

// Radio failsafe timeout (milliseconds)
constexpr uint16_t FAILSAFE_TIMEOUT_MS = 300;

// Button / switch bit assignments
constexpr uint8_t SWITCH_ARM_BIT = 0;
constexpr uint8_t SWITCH_MODE_BIT = 1;

// ------------------------------ Data types ---------------------------------

struct ControlPacket {
  uint32_t frameId;
  int16_t throttle;   // 0 -> 1000 (ESC microseconds offset from ESC_MIN_US)
  int16_t roll;       // -500 -> 500 (desired deg * 10)
  int16_t pitch;      // -500 -> 500 (desired deg * 10)
  int16_t yaw;        // -500 -> 500 (desired deg/s * 10)
  uint16_t buttons;   // momentary button bitmask
  uint8_t switches;   // toggle switches bitmask
  uint16_t pots[2];   // raw potentiometer readings (0-1023)
  uint8_t checksum;   // simple checksum
};

struct TelemetryPacket {
  uint32_t frameId;
  float roll;
  float pitch;
  float yawRate;
  float altitudeMeters;
  uint16_t batteryMv;
  uint8_t status;
};

// PID state container
struct PIDState {
  float integral;
  float previousError;
};

// Sensor raw data
struct IMURaw {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};

// ---------------------------- Global objects ------------------------------

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);

Servo motorFL;
Servo motorFR;
Servo motorRR;
Servo motorRL;

ControlPacket commandBuffer{};
TelemetryPacket telemetry{};

bool radioSignal = false;
uint32_t lastRadioMillis = 0;
bool systemArmed = false;

PIDState pidRollState{};
PIDState pidPitchState{};
PIDState pidYawState{};

float estimateRoll = 0.0f;   // degrees
float estimatePitch = 0.0f;  // degrees
float estimateYawRate = 0.0f;  // deg/s integration placeholder
float baselineAltitude = 0.0f;
float altitudeMeters = 0.0f;

// IMU calibration offsets
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
float accelBiasX = 0.0f;
float accelBiasY = 0.0f;
float accelBiasZ = 0.0f;

// MS5611 calibration coefficients
uint16_t ms5611C[6] = {0};
uint32_t lastBaroSampleMicros = 0;
bool ms5611Ready = false;

// Loop timing
uint32_t lastLoopMicros = 0;

// --------------------------- Utility functions -----------------------------

uint8_t computeChecksum(const ControlPacket &packet) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(ControlPacket) - 1; ++i) {
    sum ^= bytes[i];
  }
  return sum;
}

bool verifyChecksum(ControlPacket &packet) {
  const uint8_t expected = computeChecksum(packet);
  return expected == packet.checksum;
}

inline float degreesToRadians(float degrees) {
  return degrees * 0.01745329251f;
}

float constrainf(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

// --------------------------- Radio interface ------------------------------

void setupRadio() {
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RADIO_DATARATE);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, RADIO_ADDRESS);
  radio.startListening();
}

bool receiveControlFrame() {
  bool updated = false;
  while (radio.available()) {
    ControlPacket incoming{};
    radio.read(&incoming, sizeof(incoming));
    if (verifyChecksum(incoming)) {
      commandBuffer = incoming;
      lastRadioMillis = millis();
      updated = true;
      radioSignal = true;
    }
  }
  if (millis() - lastRadioMillis > FAILSAFE_TIMEOUT_MS) {
    radioSignal = false;
  }
  return updated;
}

void sendTelemetry() {
  telemetry.frameId = commandBuffer.frameId;
  telemetry.roll = estimateRoll;
  telemetry.pitch = estimatePitch;
  telemetry.yawRate = estimateYawRate;
  telemetry.altitudeMeters = altitudeMeters;
  telemetry.batteryMv = analogRead(A7) * (5000.0f / 1023.0f);  // assumes divider to A7
  telemetry.status = (systemArmed ? 0x01 : 0x00) | (radioSignal ? 0x02 : 0x00);

  if (radio.isAckPayloadAvailable()) {
    radio.flush_tx();
  }
  radio.writeAckPayload(1, &telemetry, sizeof(telemetry));
}

// ---------------------------- IMU interface -------------------------------

void mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("[ERR] MPU6050 not responding"));
  }

  mpuWrite(0x6B, 0x00);  // Wake up device
  delay(100);
  mpuWrite(0x1B, 0x08);  // Gyro: ±500 deg/s
  mpuWrite(0x1C, 0x10);  // Accel: ±8 g
  mpuWrite(0x1A, 0x03);  // DLPF 44 Hz
  mpuWrite(0x38, 0x01);  // Enable data ready interrupt

  pinMode(MPU_INT_PIN, INPUT);
}

void readMPU(IMURaw &imu) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  int16_t rawTemp = (Wire.read() << 8) | Wire.read();
  (void)rawTemp;
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  constexpr float accelScale = 4096.0f;   // LSB/g for ±8g
  constexpr float gyroScale = 65.5f;      // LSB/(deg/s) for ±500 deg/s

  imu.ax = ((float)rawAx / accelScale) - accelBiasX;
  imu.ay = ((float)rawAy / accelScale) - accelBiasY;
  imu.az = ((float)rawAz / accelScale) - accelBiasZ;
  imu.gx = ((float)rawGx / gyroScale) - gyroBiasX;
  imu.gy = ((float)rawGy / gyroScale) - gyroBiasY;
  imu.gz = ((float)rawGz / gyroScale) - gyroBiasZ;
}

void calibrateMPU(uint16_t samples = 2000) {
  Serial.println(F("[INFO] Calibrating IMU..."));
  float gyroSumX = 0.0f;
  float gyroSumY = 0.0f;
  float gyroSumZ = 0.0f;
  float accelSumX = 0.0f;
  float accelSumY = 0.0f;
  float accelSumZ = 0.0f;

  for (uint16_t i = 0; i < samples; ++i) {
    IMURaw imu{};
    readMPU(imu);
    gyroSumX += imu.gx;
    gyroSumY += imu.gy;
    gyroSumZ += imu.gz;
    accelSumX += imu.ax;
    accelSumY += imu.ay;
    accelSumZ += imu.az;
    delay(3);
  }

  gyroBiasX = gyroSumX / samples;
  gyroBiasY = gyroSumY / samples;
  gyroBiasZ = gyroSumZ / samples;

  accelBiasX = accelSumX / samples;
  accelBiasY = accelSumY / samples;
  accelBiasZ = (accelSumZ / samples) - 1.0f;  // Expect +1 g on Z

  Serial.println(F("[INFO] IMU calibration complete"));
}

void updateAttitude(const IMURaw &imu, float dt) {
  // Gyro integration
  float gyroRollRate = imu.gx;   // deg/s
  float gyroPitchRate = imu.gy;
  float gyroYawRate = imu.gz;

  estimateYawRate = gyroYawRate;

  float gyroRoll = estimateRoll + gyroRollRate * dt;
  float gyroPitch = estimatePitch + gyroPitchRate * dt;

  // Accelerometer angle estimate
  float accelRoll = atan2f(imu.ay, imu.az) * 57.29577951f;
  float accelPitch = atan2f(-imu.ax, sqrtf(imu.ay * imu.ay + imu.az * imu.az)) * 57.29577951f;

  estimateRoll = COMPLEMENTARY_ALPHA * gyroRoll + (1.0f - COMPLEMENTARY_ALPHA) * accelRoll;
  estimatePitch = COMPLEMENTARY_ALPHA * gyroPitch + (1.0f - COMPLEMENTARY_ALPHA) * accelPitch;
}

// ---------------------------- MS5611 interface ----------------------------

void ms5611Reset() {
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(0x1E);
  Wire.endTransmission();
  delay(3);
}

uint32_t ms5611ReadADC(uint8_t command) {
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(command);
  Wire.endTransmission();
  delayMicroseconds(10000);  // OSR4096 requires ~9.04 ms

  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(MS5611_ADDR, (uint8_t)3);

  uint32_t value = 0;
  value = (uint32_t)Wire.read() << 16;
  value |= (uint32_t)Wire.read() << 8;
  value |= Wire.read();
  return value;
}

void initMS5611() {
  ms5611Reset();
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(0xA2);
  Wire.endTransmission(false);
  Wire.requestFrom(MS5611_ADDR, (uint8_t)12);
  for (uint8_t i = 0; i < 6; ++i) {
    uint16_t coeff = (Wire.read() << 8) | Wire.read();
    ms5611C[i] = coeff;
  }
  ms5611Ready = true;
}

void updateBarometer() {
  if (!ms5611Ready) {
    return;
  }

  const uint32_t nowMicros = micros();
  if (nowMicros - lastBaroSampleMicros < 40000) {  // 25 Hz updates
    return;
  }
  lastBaroSampleMicros = nowMicros;

  uint32_t D1 = ms5611ReadADC(0x48);  // Pressure
  uint32_t D2 = ms5611ReadADC(0x58);  // Temperature

  int32_t dT = (int32_t)D2 - ((int32_t)ms5611C[4] << 8);
  int64_t OFF = ((int64_t)ms5611C[1] << 16) + (((int64_t)dT * ms5611C[3]) >> 7);
  int64_t SENS = ((int64_t)ms5611C[0] << 15) + (((int64_t)dT * ms5611C[2]) >> 8);
  int32_t P = ((D1 * SENS) >> 21) - OFF;
  P >>= 15;

  float pressure = P / 100.0f;  // mbar
  altitudeMeters = 44330.0f * (1.0f - powf(pressure / 1013.25f, 0.1903f)) - baselineAltitude;
}

void calibrateBarometer(uint16_t samples = 200) {
  Serial.println(F("[INFO] Calibrating barometer..."));
  float sum = 0.0f;
  for (uint16_t i = 0; i < samples; ++i) {
    updateBarometer();
    sum += altitudeMeters;
    delay(10);
  }
  baselineAltitude = sum / samples;
  Serial.println(F("[INFO] Barometer calibration complete"));
}

// ------------------------------ PID control -------------------------------

float updatePID(const PIDGains &gains, PIDState &state, float target, float measurement, float derivative, float dt) {
  float error = target - measurement;
  state.integral += error * dt * gains.ki;
  state.integral = constrainf(state.integral, -INTEGRAL_SATURATION, INTEGRAL_SATURATION);

  float output = gains.kp * error + state.integral + gains.kd * derivative;
  state.previousError = error;
  return output;
}

void resetPID(PIDState &state) {
  state.integral = 0.0f;
  state.previousError = 0.0f;
}

// ------------------------------ Actuators ---------------------------------

void attachMotors() {
  motorFL.attach(MOTOR_FL_PIN);
  motorFR.attach(MOTOR_FR_PIN);
  motorRR.attach(MOTOR_RR_PIN);
  motorRL.attach(MOTOR_RL_PIN);
}

void writeMotors(uint16_t fl, uint16_t fr, uint16_t rr, uint16_t rl) {
  motorFL.writeMicroseconds(fl);
  motorFR.writeMicroseconds(fr);
  motorRR.writeMicroseconds(rr);
  motorRL.writeMicroseconds(rl);
}

void disarmMotors() {
  writeMotors(ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US);
}

// ----------------------------- Core logic ---------------------------------

bool shouldArm() {
  return radioSignal && (commandBuffer.switches & (1 << SWITCH_ARM_BIT)) != 0;
}

bool inAngleMode() {
  return (commandBuffer.switches & (1 << SWITCH_MODE_BIT)) != 0;
}

void updateArmingState() {
  bool armRequested = shouldArm();
  if (armRequested && !systemArmed) {
    resetPID(pidRollState);
    resetPID(pidPitchState);
    resetPID(pidYawState);
    systemArmed = true;
    tone(BUZZER_PIN, 2000, 200);
    Serial.println(F("[INFO] System armed"));
  } else if (!armRequested && systemArmed) {
    systemArmed = false;
    disarmMotors();
    tone(BUZZER_PIN, 900, 400);
    Serial.println(F("[INFO] System disarmed"));
  }
}

float mapStickToDegrees(int16_t value) {
  // Input: -500 .. 500 => -25 .. 25 degrees
  return (value / 500.0f) * 25.0f;
}

float mapStickToYawRate(int16_t value) {
  // Input: -500 .. 500 => -150 .. 150 deg/s
  return (value / 500.0f) * 150.0f;
}

uint16_t computeMotorSignal(float throttleUs, float rollMix, float pitchMix, float yawMix, int multiplier) {
  float pulse = throttleUs + rollMix * multiplier + pitchMix * multiplier + yawMix * multiplier;
  return (uint16_t)constrainf(pulse, ESC_MIN_US, ESC_MAX_US);
}

void controlLoopStep(float dt) {
  IMURaw imu{};
  readMPU(imu);
  updateAttitude(imu, dt);
  updateBarometer();

  updateArmingState();

  if (!systemArmed || !radioSignal) {
    disarmMotors();
    digitalWrite(LED_PIN, millis() / 200 % 2);
    return;
  }

  float throttleUs = ESC_MIN_US + constrainf(commandBuffer.throttle, 0, 1000);
  float targetRoll = mapStickToDegrees(commandBuffer.roll);
  float targetPitch = mapStickToDegrees(commandBuffer.pitch);
  float targetYawRate = mapStickToYawRate(commandBuffer.yaw);

  float rollErrorDerivative = -estimateYawRate;  // Coupling compensation placeholder
  float pitchErrorDerivative = 0.0f;
  float yawDerivative = targetYawRate - estimateYawRate;

  float rollOutput = updatePID(pidRoll, pidRollState, targetRoll, estimateRoll, rollErrorDerivative, dt);
  float pitchOutput = updatePID(pidPitch, pidPitchState, targetPitch, estimatePitch, pitchErrorDerivative, dt);
  float yawOutput = updatePID(pidYaw, pidYawState, targetYawRate, estimateYawRate, yawDerivative, dt);

  // Mix for X-configuration quad
  uint16_t fl = computeMotorSignal(throttleUs, -rollOutput, +pitchOutput, +yawOutput, 1);
  uint16_t fr = computeMotorSignal(throttleUs, +rollOutput, +pitchOutput, -yawOutput, 1);
  uint16_t rr = computeMotorSignal(throttleUs, +rollOutput, -pitchOutput, +yawOutput, 1);
  uint16_t rl = computeMotorSignal(throttleUs, -rollOutput, -pitchOutput, -yawOutput, 1);

  writeMotors(fl, fr, rr, rl);
  digitalWrite(LED_PIN, HIGH);
}

// ------------------------------- Setup ------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("[BOOT] Drone flight controller starting"));

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin();
  Wire.setClock(400000);

  initMPU();
  calibrateMPU();

  initMS5611();
  calibrateBarometer();

  attachMotors();
  disarmMotors();

  setupRadio();

  lastLoopMicros = micros();
}

// -------------------------------- Loop ------------------------------------

void loop() {
  const uint32_t nowMicros = micros();
  float dt = (nowMicros - lastLoopMicros) / 1000000.0f;
  if (dt < LOOP_DT) {
    return;
  }
  lastLoopMicros = nowMicros;

  receiveControlFrame();
  controlLoopStep(dt);
  sendTelemetry();
}
