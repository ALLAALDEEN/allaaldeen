/**
 * Flight_Controller.ino
 *
 * Arduino Nano based quadcopter flight controller.
 * Hardware:
 *  - NRF24L01 (CE -> D4, CSN -> D10)
 *  - MPU6050 IMU (INT -> D2)
 *  - GY-36 MS5611 barometer
 *  - ESC outputs: FL->D3, FR->D5, RR->D6, RL->D7
 *  - Buzzer -> D8
 *  - Optional status LED -> D13
 *
 * This sketch implements:
 *  - Complementary filter attitude estimation (pitch/roll)
 *  - Simple gyro-based yaw hold
 *  - PID attitude stabilization
 *  - NRF24 based RC input with checksum and failsafe
 *  - Basic altitude estimation via MS5611 (for telemetry/use in future features)
 *
 * NOTE: Ensure ESC signal ground is shared with flight controller ground.
 *       Calibrate sensors and ESCs before first flight. See documentation.
 */

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// --- Radio payload definition (must match transmitter) ----------------------
struct __attribute__((packed)) RadioPacket {
  uint16_t throttle;
  uint16_t yaw;
  uint16_t pitch;
  uint16_t roll;
  uint16_t aux1;
  uint16_t aux2;
  uint16_t pot1;
  uint16_t pot2;
  uint8_t buttons;
  uint8_t switches;
  uint16_t checksum;
};

// --- Pin assignments --------------------------------------------------------
constexpr uint8_t PIN_RF_CE  = 4;
constexpr uint8_t PIN_RF_CSN = 10;
constexpr uint8_t PIN_IMU_INT = 2;

constexpr uint8_t PIN_ESC_FL = 3;
constexpr uint8_t PIN_ESC_FR = 5;
constexpr uint8_t PIN_ESC_RR = 6;
constexpr uint8_t PIN_ESC_RL = 7;

constexpr uint8_t PIN_BUZZER = 8;
constexpr uint8_t PIN_LED    = LED_BUILTIN;

// --- Flight constants -------------------------------------------------------
constexpr uint32_t LOOP_HZ        = 250;
constexpr float    LOOP_DT        = 1.0f / LOOP_HZ;
constexpr float    COMPLEMENTARY_ALPHA = 0.98f;

constexpr uint16_t RC_MIN = 1000;
constexpr uint16_t RC_MAX = 2000;
constexpr uint16_t RC_MID = (RC_MIN + RC_MAX) / 2;

constexpr uint32_t FAILSAFE_TIMEOUT_MS = 500;
constexpr uint16_t THROTTLE_ARM_THRESHOLD = 1050;

// PID gains (start conservative; tune per airframe)
struct PIDGains {
  float kp;
  float ki;
  float kd;
};

const PIDGains PID_PITCH_GAINS = {4.0f, 0.02f, 1.2f};
const PIDGains PID_ROLL_GAINS  = {4.0f, 0.02f, 1.2f};
const PIDGains PID_YAW_GAINS   = {3.5f, 0.01f, 0.0f};

// --- Global state -----------------------------------------------------------
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
const byte RADIO_ADDRESS[6] = "FC01";

Servo escFL, escFR, escRR, escRL;

volatile bool imuInterrupt = false;

// Gyro/Accel bias estimates
float gyroBias[3] = {0, 0, 0};
float accelBias[3] = {0, 0, 0};

// Current attitude
float pitchDeg = 0.0f;
float rollDeg  = 0.0f;
float yawRateDeg = 0.0f; // using gyro z-rate

// Altitude readings
float baroAltitudeMeters = 0.0f;

// RC command cache
RadioPacket lastPacket{};
uint32_t lastPacketMs = 0;
bool radioLinkActive = false;
bool armed = false;

// PID controller structure
struct PIDController {
  PIDGains gains;
  float integral;
  float prevError;
  float output;

  void reset() {
    integral = 0.0f;
    prevError = 0.0f;
    output = 0.0f;
  }

  float update(float target, float measurement, float dt) {
    float error = target - measurement;
    integral += error * dt;
    integral = constrain(integral, -200.0f, 200.0f); // windup guard

    float derivative = (error - prevError) / dt;
    prevError = error;

    output = (gains.kp * error) + (gains.ki * integral) + (gains.kd * derivative);
    return output;
  }
};

PIDController pidPitch{PID_PITCH_GAINS, 0.0f, 0.0f, 0.0f};
PIDController pidRoll{PID_ROLL_GAINS, 0.0f, 0.0f, 0.0f};
PIDController pidYaw{PID_YAW_GAINS, 0.0f, 0.0f, 0.0f};

// --- Utility functions ------------------------------------------------------
void onImuInterrupt() {
  imuInterrupt = true;
}

int16_t read16(int addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)2);
  int16_t value = (Wire.read() << 8) | Wire.read();
  return value;
}

uint8_t read8(int addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.read();
}

void write8(int addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

// --- MPU6050 interface ------------------------------------------------------
namespace MPU6050 {
const uint8_t ADDRESS = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_SMPLRT_DIV = 0x19;
const uint8_t REG_CONFIG = 0x1A;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_INT_ENABLE = 0x38;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
}

void setupIMU() {
  write8(MPU6050::ADDRESS, MPU6050::REG_PWR_MGMT_1, 0x00); // wake up
  delay(100);
  write8(MPU6050::ADDRESS, MPU6050::REG_SMPLRT_DIV, 0x00); // 1 kHz
  write8(MPU6050::ADDRESS, MPU6050::REG_CONFIG, 0x03);     // DLPF 44Hz
  write8(MPU6050::ADDRESS, MPU6050::REG_GYRO_CONFIG, 0x08);   // ±500°/s
  write8(MPU6050::ADDRESS, MPU6050::REG_ACCEL_CONFIG, 0x00);  // ±2g
  write8(MPU6050::ADDRESS, MPU6050::REG_INT_ENABLE, 0x01);    // data ready

  attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), onImuInterrupt, RISING);

  // Calibration
  const uint16_t samples = 2000;
  float gyro[3] = {0, 0, 0};
  float accel[3] = {0, 0, 0};
  for (uint16_t i = 0; i < samples; ++i) {
    while (!imuInterrupt) { /* wait */ }
    imuInterrupt = false;

    Wire.beginTransmission(MPU6050::ADDRESS);
    Wire.write(MPU6050::REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050::ADDRESS, (uint8_t)14);
    int16_t rawAx = (Wire.read() << 8) | Wire.read();
    int16_t rawAy = (Wire.read() << 8) | Wire.read();
    int16_t rawAz = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // temp
    int16_t rawGx = (Wire.read() << 8) | Wire.read();
    int16_t rawGy = (Wire.read() << 8) | Wire.read();
    int16_t rawGz = (Wire.read() << 8) | Wire.read();

    gyro[0] += rawGx;
    gyro[1] += rawGy;
    gyro[2] += rawGz;
    accel[0] += rawAx;
    accel[1] += rawAy;
    accel[2] += rawAz;
    delayMicroseconds(500);
  }

  for (int i = 0; i < 3; ++i) {
    gyroBias[i] = gyro[i] / samples;
    accelBias[i] = accel[i] / samples;
  }
  // For the accelerometer Z bias, subtract 1g (assuming sensor resting flat)
  accelBias[2] -= 16384.0f;
}

void readIMU(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  while (!imuInterrupt) { /* wait for data ready */ }
  imuInterrupt = false;

  Wire.beginTransmission(MPU6050::ADDRESS);
  Wire.write(MPU6050::REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050::ADDRESS, (uint8_t)14);

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Skip temp
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  ax = (rawAx - accelBias[0]) / 16384.0f;
  ay = (rawAy - accelBias[1]) / 16384.0f;
  az = (rawAz - accelBias[2]) / 16384.0f;

  gx = (rawGx - gyroBias[0]) / 65.5f; // deg/s for ±500dps
  gy = (rawGy - gyroBias[1]) / 65.5f;
  gz = (rawGz - gyroBias[2]) / 65.5f;
}

// --- MS5611 interface -------------------------------------------------------
namespace MS5611 {
const uint8_t ADDRESS = 0x77;
const uint8_t CMD_RESET = 0x1E;
const uint8_t CMD_PROM_READ = 0xA0;
const uint8_t CMD_CONVERT_D1_OSR4096 = 0x48;
const uint8_t CMD_CONVERT_D2_OSR4096 = 0x58;
const uint8_t CMD_ADC_READ = 0x00;
}

uint16_t ms5611Cal[6] = {0};

void setupBarometer() {
  write8(MS5611::ADDRESS, MS5611::CMD_RESET, 0);
  delay(3);
  for (uint8_t i = 0; i < 6; ++i) {
    Wire.beginTransmission(MS5611::ADDRESS);
    Wire.write(MS5611::CMD_PROM_READ + (i + 1) * 2);
    Wire.endTransmission(false);
    Wire.requestFrom(MS5611::ADDRESS, (uint8_t)2);
    ms5611Cal[i] = (Wire.read() << 8) | Wire.read();
  }
}

uint32_t readADC(uint8_t command) {
  Wire.beginTransmission(MS5611::ADDRESS);
  Wire.write(command);
  Wire.endTransmission();
  delay(10); // max conversion time for OSR4096
  Wire.beginTransmission(MS5611::ADDRESS);
  Wire.write(MS5611::CMD_ADC_READ);
  Wire.endTransmission(false);
  Wire.requestFrom(MS5611::ADDRESS, (uint8_t)3);
  uint32_t value = 0;
  value = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
  return value;
}

bool readBarometer(float& temperatureC, float& pressurePa, float& altitudeM) {
  if (ms5611Cal[0] == 0) {
    return false;
  }

  uint32_t D1 = readADC(MS5611::CMD_CONVERT_D1_OSR4096);
  uint32_t D2 = readADC(MS5611::CMD_CONVERT_D2_OSR4096);

  int32_t dT = D2 - ((int32_t)ms5611Cal[4] << 8);
  int32_t TEMP = 2000 + ((int64_t)dT * ms5611Cal[5]) / 8388608;
  int64_t OFF = ((int64_t)ms5611Cal[1] << 16) + ((int64_t)ms5611Cal[3] * dT) / 128;
  int64_t SENS = ((int64_t)ms5611Cal[0] << 15) + ((int64_t)ms5611Cal[2] * dT) / 256;
  int32_t P = (((int64_t)D1 * SENS) / 2097152 - OFF) / 32768;

  temperatureC = TEMP / 100.0f;
  pressurePa = P;

  // Barometric formula (approximate, assuming sea-level pressure 101325 Pa)
  altitudeM = 44330.0f * (1.0f - pow(pressurePa / 101325.0f, 0.1903f));
  return true;
}

// --- Radio handling ---------------------------------------------------------
bool validatePacket(const RadioPacket& packet) {
  uint16_t checksum = 0xFFFF;
  checksum ^= packet.throttle;
  checksum ^= packet.yaw;
  checksum ^= packet.pitch;
  checksum ^= packet.roll;
  checksum ^= packet.aux1;
  checksum ^= packet.aux2;
  checksum ^= packet.pot1;
  checksum ^= packet.pot2;
  checksum ^= packet.buttons;
  checksum ^= packet.switches;
  return checksum == packet.checksum;
}

bool readRadio() {
  bool packetReceived = false;
  while (radio.available()) {
    radio.read(&lastPacket, sizeof(lastPacket));
    packetReceived = true;
  }

  if (packetReceived && validatePacket(lastPacket)) {
    lastPacketMs = millis();
    radioLinkActive = true;
  }
  return packetReceived;
}

// Convert RC command to degrees setpoint (centered around zero)
float rcToAngle(uint16_t value, float maxAngle = 25.0f) {
  int16_t centered = static_cast<int16_t>(value) - RC_MID;
  return (centered / 500.0f) * maxAngle; // with RC range 1000-2000 => ±1
}

float rcToYawRate(uint16_t value, float maxRate = 180.0f) {
  int16_t centered = static_cast<int16_t>(value) - RC_MID;
  return (centered / 500.0f) * maxRate;
}

// --- ESC handling -----------------------------------------------------------
void writeMotors(float fl, float fr, float rr, float rl) {
  fl = constrain(fl, RC_MIN, RC_MAX);
  fr = constrain(fr, RC_MIN, RC_MAX);
  rr = constrain(rr, RC_MIN, RC_MAX);
  rl = constrain(rl, RC_MIN, RC_MAX);

  escFL.writeMicroseconds(static_cast<int>(fl));
  escFR.writeMicroseconds(static_cast<int>(fr));
  escRR.writeMicroseconds(static_cast<int>(rr));
  escRL.writeMicroseconds(static_cast<int>(rl));
}

void stopMotors() {
  writeMotors(RC_MIN, RC_MIN, RC_MIN, RC_MIN);
}

// --- Setup & Loop -----------------------------------------------------------
void setup() {
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LED, LOW);

  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);
  Serial.println(F("Flight Controller Booting..."));

  setupIMU();
  setupBarometer();

  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(108);
  radio.openReadingPipe(1, RADIO_ADDRESS);
  radio.startListening();

  escFL.attach(PIN_ESC_FL);
  escFR.attach(PIN_ESC_FR);
  escRR.attach(PIN_ESC_RR);
  escRL.attach(PIN_ESC_RL);
  stopMotors();

  delay(500);
  Serial.println(F("Calibration complete. Waiting for arm command."));
}

void updateAttitude(float dt) {
  float ax, ay, az, gx, gy, gz;
  readIMU(ax, ay, az, gx, gy, gz);
  yawRateDeg = gz;

  float pitchAcc = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / PI;
  float rollAcc  = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

  pitchDeg = COMPLEMENTARY_ALPHA * (pitchDeg + gx * dt) + (1.0f - COMPLEMENTARY_ALPHA) * pitchAcc;
  rollDeg  = COMPLEMENTARY_ALPHA * (rollDeg + gy * dt) + (1.0f - COMPLEMENTARY_ALPHA) * rollAcc;
}

void loop() {
  static uint32_t lastLoopMicros = micros();
  uint32_t nowMicros = micros();
  float dt = (nowMicros - lastLoopMicros) / 1e6f;
  if (dt <= 0.0005f) {
    return; // wait for next cycle to avoid crazy dt
  }
  lastLoopMicros = nowMicros;

  if (dt > 0.02f) {
    dt = 0.02f; // clamp
  }

  updateAttitude(dt);

  static uint32_t lastBaroMs = 0;
  if (millis() - lastBaroMs > 50) { // 20 Hz
    float t, p, alt;
    if (readBarometer(t, p, alt)) {
      baroAltitudeMeters = alt;
    }
    lastBaroMs = millis();
  }

  readRadio();
  uint32_t nowMs = millis();
  if (radioLinkActive && (nowMs - lastPacketMs) > FAILSAFE_TIMEOUT_MS) {
    radioLinkActive = false;
    armed = false;
    stopMotors();
    tone(PIN_BUZZER, 1200, 100);
    Serial.println(F("Failsafe triggered: radio timeout"));
  }

  bool armSwitchActive = (lastPacket.switches & 0x01) != 0;
  if (!armed) {
    if (armSwitchActive && lastPacket.throttle < THROTTLE_ARM_THRESHOLD && radioLinkActive) {
      armed = true;
      pidPitch.reset();
      pidRoll.reset();
      pidYaw.reset();
      tone(PIN_BUZZER, 2000, 200);
      Serial.println(F("Motors armed"));
      delay(50);
    }
  } else {
    if (!armSwitchActive || lastPacket.throttle < RC_MIN + 10) {
      armed = false;
      stopMotors();
      tone(PIN_BUZZER, 1000, 150);
      Serial.println(F("Motors disarmed"));
    }
  }

  digitalWrite(PIN_LED, armed ? HIGH : LOW);

  if (!armed) {
    stopMotors();
    return;
  }

  float throttle = constrain(lastPacket.throttle, RC_MIN, RC_MAX);
  float pitchSetpoint = rcToAngle(lastPacket.pitch);
  float rollSetpoint  = rcToAngle(lastPacket.roll);
  float yawRateSetpoint = rcToYawRate(lastPacket.yaw);

  float pitchTerm = pidPitch.update(pitchSetpoint, pitchDeg, dt);
  float rollTerm  = pidRoll.update(rollSetpoint, rollDeg, dt);
  float yawTerm   = pidYaw.update(yawRateSetpoint, yawRateDeg, dt);

  float motorFL = throttle + pitchTerm + rollTerm - yawTerm;
  float motorFR = throttle + pitchTerm - rollTerm + yawTerm;
  float motorRR = throttle - pitchTerm - rollTerm - yawTerm;
  float motorRL = throttle - pitchTerm + rollTerm + yawTerm;

  // Constrain and output
  writeMotors(motorFL, motorFR, motorRR, motorRL);

  // Optional debug output at low rate
  static uint32_t lastDebugMs = 0;
  if (Serial && millis() - lastDebugMs > 100) {
    lastDebugMs = millis();
    Serial.print(F("P:"));
    Serial.print(pitchDeg, 2);
    Serial.print(F(" R:"));
    Serial.print(rollDeg, 2);
    Serial.print(F(" T:"));
    Serial.print(throttle);
    Serial.print(F(" Alt:"));
    Serial.print(baroAltitudeMeters, 1);
    Serial.print(F(" PID("));
    Serial.print(pitchTerm, 1);
    Serial.print(',');
    Serial.print(rollTerm, 1);
    Serial.print(',');
    Serial.print(yawTerm, 1);
    Serial.println(')');
  }
}
