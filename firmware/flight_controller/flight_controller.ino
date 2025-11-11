#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Servo.h>
#include <RF24.h>
#include "MPU6050.h"

// ---------------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------------
constexpr uint8_t PIN_ESC_FL = 3;   // Front-left
constexpr uint8_t PIN_ESC_FR = 5;   // Front-right
constexpr uint8_t PIN_ESC_RR = 6;   // Rear-right
constexpr uint8_t PIN_ESC_RL = 9;   // Rear-left
constexpr uint8_t PIN_BUZZER = 4;
constexpr uint8_t PIN_STATUS_LED = 10;

constexpr uint8_t PIN_RADIO_CE = 7;
constexpr uint8_t PIN_RADIO_CSN = 8;

constexpr uint8_t MPU_INTERRUPT_PIN = 2;

constexpr uint8_t EEPROM_SIGNATURE_ADDR = 0;
constexpr uint8_t EEPROM_CAL_ADDR = 4;

// ---------------------------------------------------------------------------
// Motor configuration
// ---------------------------------------------------------------------------
constexpr int16_t ESC_MIN_US = 1000;
constexpr int16_t ESC_IDLE_US = 1120;
constexpr int16_t ESC_MAX_US = 2000;
constexpr uint16_t MOTOR_SPIN_RAMP_TIME_MS = 2000;

// ---------------------------------------------------------------------------
// Control loop rates
// ---------------------------------------------------------------------------
constexpr uint16_t LOOP_IMU_HZ = 1000;
constexpr uint16_t LOOP_CONTROL_HZ = 400;
constexpr uint16_t LOOP_RADIO_HZ = 200;
constexpr uint16_t LOOP_TELEMETRY_HZ = 20;

// ---------------------------------------------------------------------------
// RF24 configuration
// ---------------------------------------------------------------------------
constexpr byte RADIO_ADDRESS[6] = "FC001";
constexpr byte RADIO_ADDRESS_BACK[6] = "TX001";
constexpr uint8_t RADIO_CHANNEL = 110;
constexpr uint8_t RADIO_PAYLOAD_SIZE = 32;

// ---------------------------------------------------------------------------
// Flight states
// ---------------------------------------------------------------------------
enum class FlightState : uint8_t {
  POWER_ON = 0,
  LINK_WAIT,
  SAFE_KILL,
  CALIBRATING,
  READY,
  MOTOR_SPIN,
  ARMED
};

// ---------------------------------------------------------------------------
// Calibration structure
// ---------------------------------------------------------------------------
struct CalibrationData {
  float accelOffset[3];
  float gyroOffset[3];
  int16_t escMin;
  int16_t escMax;
  uint32_t crc;
};

CalibrationData calData;

// ---------------------------------------------------------------------------
// Radio packet structures
// ---------------------------------------------------------------------------
struct __attribute__((packed)) RadioCommand {
  uint8_t flags;
  uint16_t throttle;
  int16_t roll;
  int16_t pitch;
  int16_t yawRate;
  uint16_t seq;
  uint16_t checksum;
};

struct __attribute__((packed)) TelemetryFrame {
  uint8_t state;
  int16_t rollDeg;
  int16_t pitchDeg;
  int16_t yawDeg;
  int16_t altitudeCm;
  uint16_t batteryMv;
  uint16_t linkQuality;
  uint16_t lastSeq;
  uint16_t checksum;
};

// Flag bit definitions
constexpr uint8_t FLAG_ARMED = 0x01;
constexpr uint8_t FLAG_KILL = 0x02;
constexpr uint8_t FLAG_CALIBRATE = 0x04;
constexpr uint8_t FLAG_MOTOR_SPIN = 0x08;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);
MPU6050 imu;

Servo escFL;
Servo escFR;
Servo escRR;
Servo escRL;

FlightState flightState = FlightState::POWER_ON;

RadioCommand currentCommand = {};
TelemetryFrame telemetry = {};

uint32_t lastIMUUpdateUs = 0;
uint32_t lastControlUpdateUs = 0;
uint32_t lastRadioUpdateUs = 0;
uint32_t lastTelemetryUs = 0;
uint32_t lastRadioPacketMs = 0;

float angleRoll = 0.0f;
float anglePitch = 0.0f;
float angleYaw = 0.0f;
float altitudeEstimate = 0.0f;
float verticalVelocity = 0.0f;
float gyroRate[3] = {0.0f, 0.0f, 0.0f};

uint32_t motorSpinStartMs = 0;
bool linkEstablished = false;
bool prevCalibrateFlag = false;

// PID terms
struct PID {
  float kp;
  float ki;
  float kd;
  float integrator;
  float prevError;
  float integratorLimit;
};

PID pidRoll{4.0f, 0.02f, 20.0f, 0.0f, 0.0f, 80.0f};
PID pidPitch{4.0f, 0.02f, 20.0f, 0.0f, 0.0f, 80.0f};
PID pidYaw{3.0f, 0.01f, 8.0f, 0.0f, 0.0f, 50.0f};

// ---------------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------------
uint32_t crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; ++i) {
    uint8_t byte = data[i];
    crc ^= byte;
    for (uint8_t j = 0; j < 8; ++j) {
      uint32_t mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
  }
  return ~crc;
}

uint16_t checksum16(const uint8_t* data, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; ++i) {
    sum += data[i];
  }
  sum = (sum & 0xFFFF) + (sum >> 16);
  sum = (sum & 0xFFFF) + (sum >> 16);
  return static_cast<uint16_t>(~sum);
}

void flashStatus(uint8_t times, uint16_t delayMs) {
  for (uint8_t i = 0; i < times; ++i) {
    digitalWrite(PIN_STATUS_LED, HIGH);
    delay(delayMs);
    digitalWrite(PIN_STATUS_LED, LOW);
    delay(delayMs);
  }
}

void setBuzzer(bool on) {
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
}

void setAllMotors(int16_t pulseUs) {
  pulseUs = constrain(pulseUs, ESC_MIN_US, ESC_MAX_US);
  escFL.writeMicroseconds(pulseUs);
  escFR.writeMicroseconds(pulseUs);
  escRR.writeMicroseconds(pulseUs);
  escRL.writeMicroseconds(pulseUs);
}

void stopMotors() {
  setAllMotors(ESC_MIN_US);
}

// ---------------------------------------------------------------------------
// EEPROM persistence
// ---------------------------------------------------------------------------
bool loadCalibration() {
  uint32_t signature = 0;
  EEPROM.get(EEPROM_SIGNATURE_ADDR, signature);
  if (signature != 0xA5FCA55A) {
    return false;
  }

  EEPROM.get(EEPROM_CAL_ADDR, calData);
  uint32_t crc = calData.crc;
  calData.crc = 0;

  if (crc32(reinterpret_cast<uint8_t*>(&calData), sizeof(CalibrationData)) != crc) {
    return false;
  }
  calData.crc = crc;
  return true;
}

void saveCalibration() {
  calData.escMin = ESC_MIN_US;
  calData.escMax = ESC_MAX_US;
  calData.crc = 0;
  calData.crc = crc32(reinterpret_cast<uint8_t*>(&calData), sizeof(CalibrationData));
  EEPROM.put(EEPROM_CAL_ADDR, calData);
  uint32_t signature = 0xA5FCA55A;
  EEPROM.put(EEPROM_SIGNATURE_ADDR, signature);
}

// ---------------------------------------------------------------------------
// PID controller
// ---------------------------------------------------------------------------
float updatePID(PID& pid, float target, float measurement, float dt) {
  float error = target - measurement;
  pid.integrator += error * dt * pid.ki;
  pid.integrator = constrain(pid.integrator, -pid.integratorLimit, pid.integratorLimit);
  float derivative = (error - pid.prevError) / dt;
  pid.prevError = error;
  return pid.kp * error + pid.integrator + pid.kd * derivative;
}

// ---------------------------------------------------------------------------
// IMU handling
// ---------------------------------------------------------------------------
void initIMU() {
  Wire.begin();
  imu.initialize();
  imu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
}

void performImuCalibration() {
  const uint16_t samples = 2000;
  float accelAccum[3] = {0, 0, 0};
  float gyroAccum[3] = {0, 0, 0};

  for (uint16_t i = 0; i < samples; ++i) {
    VectorInt16 accelRaw;
    VectorInt16 gyroRaw;
    imu.getMotion6(&accelRaw.x, &accelRaw.y, &accelRaw.z, &gyroRaw.x, &gyroRaw.y, &gyroRaw.z);
    accelAccum[0] += accelRaw.x;
    accelAccum[1] += accelRaw.y;
    accelAccum[2] += accelRaw.z - 16384.0f; // subtract 1g
    gyroAccum[0] += gyroRaw.x;
    gyroAccum[1] += gyroRaw.y;
    gyroAccum[2] += gyroRaw.z;
    delay(2);
  }

  for (uint8_t i = 0; i < 3; ++i) {
    calData.accelOffset[i] = accelAccum[i] / samples;
    calData.gyroOffset[i] = gyroAccum[i] / samples;
  }

  saveCalibration();
}

void calibrateEsc() {
  Serial.println(F("[Cal] ESC calibration: sending MAX throttle in 2 seconds. Disconnect LiPo now."));
  flashStatus(5, 60);
  for (int i = 3; i >= 1; --i) {
    Serial.print(F("[Cal] Connect LiPo in "));
    Serial.print(i);
    Serial.println(F("..."));
    setBuzzer(true);
    delay(80);
    setBuzzer(false);
    delay(420);
  }

  Serial.println(F("[Cal] Sending MAX throttle..."));
  setAllMotors(ESC_MAX_US);
  delay(2000);

  Serial.println(F("[Cal] Sending MIN throttle..."));
  setAllMotors(ESC_MIN_US);
  delay(2000);

  stopMotors();
  Serial.println(F("[Cal] ESC calibration complete."));
}

void performSystemCalibration() {
  setBuzzer(true);
  delay(120);
  setBuzzer(false);
  Serial.println(F("[Cal] Starting system calibration."));
  performImuCalibration();
  calibrateEsc();
  flashStatus(4, 80);
  Serial.println(F("[Cal] Calibration finished and saved."));
}

bool readIMU(float& gx, float& gy, float& gz, float& ax, float& ay, float& az) {
  VectorInt16 accelRaw;
  VectorInt16 gyroRaw;

  imu.getMotion6(&accelRaw.x, &accelRaw.y, &accelRaw.z, &gyroRaw.x, &gyroRaw.y, &gyroRaw.z);

  ax = (accelRaw.x - calData.accelOffset[0]) / 16384.0f; // g
  ay = (accelRaw.y - calData.accelOffset[1]) / 16384.0f;
  az = (accelRaw.z - calData.accelOffset[2]) / 16384.0f;

  gx = (gyroRaw.x - calData.gyroOffset[0]) / 65.5f; // deg/s
  gy = (gyroRaw.y - calData.gyroOffset[1]) / 65.5f;
  gz = (gyroRaw.z - calData.gyroOffset[2]) / 65.5f;

  return true;
}

void updateAttitude(float dt) {
  float gx, gy, gz, ax, ay, az;
  if (!readIMU(gx, gy, gz, ax, ay, az)) {
    return;
  }

  // Integrate gyro
  angleRoll += gx * dt;
  anglePitch += gy * dt;
  angleYaw += gz * dt;

  if (angleYaw > 180.0f) angleYaw -= 360.0f;
  if (angleYaw < -180.0f) angleYaw += 360.0f;

  // Accelerometer-based angles
  float rollAcc = atan2f(ay, az) * RAD_TO_DEG;
  float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  const float alpha = 0.98f;
  angleRoll = alpha * angleRoll + (1 - alpha) * rollAcc;
  anglePitch = alpha * anglePitch + (1 - alpha) * pitchAcc;

  // Vertical acceleration for altitude estimate
  float accZ = (az - 1.0f) * 9.81f; // remove gravity
  const float altAlpha = 0.90f;
  verticalVelocity = verticalVelocity + accZ * dt;
  altitudeEstimate = altAlpha * (altitudeEstimate + verticalVelocity * dt) + (1 - altAlpha) * altitudeEstimate;
  gyroRate[0] = gx;
  gyroRate[1] = gy;
  gyroRate[2] = gz;
}

// ---------------------------------------------------------------------------
// Radio handling
// ---------------------------------------------------------------------------
bool receiveRadioCommand(RadioCommand& cmd) {
  if (!radio.available()) {
    return false;
  }
  radio.read(&cmd, sizeof(RadioCommand));

  uint16_t receivedChecksum = cmd.checksum;
  cmd.checksum = 0;
  if (checksum16(reinterpret_cast<uint8_t*>(&cmd), sizeof(RadioCommand)) != receivedChecksum) {
    return false;
  }
  cmd.checksum = receivedChecksum;
  lastRadioPacketMs = millis();
  if (!linkEstablished) {
    linkEstablished = true;
    flashStatus(2, 60);
    setBuzzer(true);
    delay(70);
    setBuzzer(false);
  }
  return true;
}

void sendTelemetry() {
  telemetry.state = static_cast<uint8_t>(flightState);
  telemetry.rollDeg = static_cast<int16_t>(angleRoll * 10.0f);
  telemetry.pitchDeg = static_cast<int16_t>(anglePitch * 10.0f);
  telemetry.yawDeg = static_cast<int16_t>(angleYaw * 10.0f);
  telemetry.altitudeCm = static_cast<int16_t>(altitudeEstimate * 100.0f);
  telemetry.batteryMv = 0; // TODO: add voltage sensing
  telemetry.linkQuality = constrain((millis() - lastRadioPacketMs), 0, 1000);
  telemetry.lastSeq = currentCommand.seq;
  telemetry.checksum = 0;
  telemetry.checksum = checksum16(reinterpret_cast<uint8_t*>(&telemetry), sizeof(TelemetryFrame));

  radio.stopListening();
  radio.write(&telemetry, sizeof(TelemetryFrame));
  radio.startListening();
}

// ---------------------------------------------------------------------------
// Command processing and failsafe
// ---------------------------------------------------------------------------
bool radioTimeout() {
  return (millis() - lastRadioPacketMs) > 200;
}

void enterFailsafe() {
  flightState = FlightState::SAFE_KILL;
  stopMotors();
  setBuzzer(false);
}

void applyMotorMix(float throttle, float rollCmd, float pitchCmd, float yawCmd) {
  // Basic X quad mix
  float motor[4];
  motor[0] = throttle + pitchCmd + rollCmd - yawCmd; // FL (CCW)
  motor[1] = throttle + pitchCmd - rollCmd + yawCmd; // FR (CW)
  motor[2] = throttle - pitchCmd - rollCmd - yawCmd; // RR (CCW)
  motor[3] = throttle - pitchCmd + rollCmd + yawCmd; // RL (CW)

  for (uint8_t i = 0; i < 4; ++i) {
    motor[i] = constrain(motor[i], ESC_MIN_US, ESC_MAX_US);
  }

  escFL.writeMicroseconds(static_cast<int>(motor[0]));
  escFR.writeMicroseconds(static_cast<int>(motor[1]));
  escRR.writeMicroseconds(static_cast<int>(motor[2]));
  escRL.writeMicroseconds(static_cast<int>(motor[3]));
}

// ---------------------------------------------------------------------------
// State machine helpers
// ---------------------------------------------------------------------------
bool throttleSafe() {
  return currentCommand.throttle <= (ESC_MIN_US + 10);
}

void processStateMachine() {
  bool killRequested = currentCommand.flags & FLAG_KILL;
  bool armRequested = currentCommand.flags & FLAG_ARMED;
  bool motorSpinRequested = currentCommand.flags & FLAG_MOTOR_SPIN;
  bool calibrateRequested = currentCommand.flags & FLAG_CALIBRATE;
  bool calibrateEdge = calibrateRequested && !prevCalibrateFlag;

  switch (flightState) {
    case FlightState::POWER_ON:
      stopMotors();
      if (linkEstablished) {
        flightState = FlightState::LINK_WAIT;
      }
      break;

    case FlightState::LINK_WAIT:
      stopMotors();
      if (killRequested) {
        flightState = FlightState::SAFE_KILL;
        setBuzzer(true);  // Indicate link + kill safe
      }
      break;

    case FlightState::SAFE_KILL:
      stopMotors();
      if (!killRequested && throttleSafe()) {
        flightState = FlightState::READY;
        setBuzzer(false);
      }
      if (calibrateEdge) {
        performSystemCalibration();
        linkEstablished = true; // keep link flag
      }
      break;

    case FlightState::READY:
      stopMotors();
      if (calibrateEdge) {
        flightState = FlightState::CALIBRATING;
        performSystemCalibration();
        flightState = FlightState::READY;
        flashStatus(3, 80);
      } else if (killRequested) {
        flightState = FlightState::SAFE_KILL;
        setBuzzer(false);
      } else if (motorSpinRequested && throttleSafe()) {
        flightState = FlightState::MOTOR_SPIN;
        motorSpinStartMs = millis();
      } else if (armRequested && throttleSafe()) {
        flightState = FlightState::ARMED;
      }
      break;

    case FlightState::CALIBRATING:
      stopMotors();
      if (!calibrateRequested) {
        flightState = FlightState::READY;
      }
      break;

    case FlightState::MOTOR_SPIN: {
      uint32_t elapsed = millis() - motorSpinStartMs;
      if (killRequested) {
        flightState = FlightState::SAFE_KILL;
        stopMotors();
        break;
      }

      if (!motorSpinRequested) {
        flightState = FlightState::READY;
        stopMotors();
        break;
      }

      int16_t target = map(constrain(elapsed, 0UL, MOTOR_SPIN_RAMP_TIME_MS),
                           0, MOTOR_SPIN_RAMP_TIME_MS,
                           ESC_MIN_US, ESC_IDLE_US);
      setAllMotors(target);

      if (elapsed >= MOTOR_SPIN_RAMP_TIME_MS && armRequested) {
        flightState = FlightState::ARMED;
      }
      break;
    }

    case FlightState::ARMED:
      if (killRequested) {
        flightState = FlightState::SAFE_KILL;
        stopMotors();
        break;
      }

      if (!armRequested) {
        flightState = FlightState::READY;
        stopMotors();
        break;
      }

      if (radioTimeout()) {
        enterFailsafe();
      }
      break;
  }

  prevCalibrateFlag = calibrateRequested;
}

// ---------------------------------------------------------------------------
// Motor control loop
// ---------------------------------------------------------------------------
void updateMotorOutputs(float dt) {
  float throttle = currentCommand.throttle;
  throttle = constrain(throttle, ESC_MIN_US, ESC_MAX_US);

  float rollTarget = currentCommand.roll / 10.0f;   // degrees
  float pitchTarget = currentCommand.pitch / 10.0f;
  float yawRateTarget = currentCommand.yawRate / 10.0f;

  float rollCorrection = updatePID(pidRoll, rollTarget, angleRoll, dt);
  float pitchCorrection = updatePID(pidPitch, pitchTarget, anglePitch, dt);
  float yawCorrection = updatePID(pidYaw, yawRateTarget, gyroRate[2], dt);

  applyMotorMix(throttle, rollCorrection, pitchCorrection, yawCorrection);
}

// ---------------------------------------------------------------------------
// Setup & main loop
// ---------------------------------------------------------------------------
void setup() {
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(MPU_INTERRUPT_PIN, INPUT);
  setBuzzer(false);
  digitalWrite(PIN_STATUS_LED, LOW);

  Serial.begin(115200);

  Serial.println(F("Quadcopter Flight Controller Booting..."));

  escFL.attach(PIN_ESC_FL, ESC_MIN_US, ESC_MAX_US);
  escFR.attach(PIN_ESC_FR, ESC_MIN_US, ESC_MAX_US);
  escRR.attach(PIN_ESC_RR, ESC_MIN_US, ESC_MAX_US);
  escRL.attach(PIN_ESC_RL, ESC_MIN_US, ESC_MAX_US);
  stopMotors();

  initIMU();
  if (!loadCalibration()) {
    Serial.println(F("No calibration data. Performing IMU calibration..."));
    performImuCalibration();
  }

  radio.begin();
  radio.setChannel(RADIO_CHANNEL);
  radio.setPayloadSize(RADIO_PAYLOAD_SIZE);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setRetries(3, 5);
  radio.openReadingPipe(1, RADIO_ADDRESS);
  radio.openWritingPipe(RADIO_ADDRESS_BACK);
  radio.startListening();

  lastIMUUpdateUs = micros();
  lastControlUpdateUs = lastIMUUpdateUs;
  lastRadioUpdateUs = lastIMUUpdateUs;
  lastTelemetryUs = lastIMUUpdateUs;
  lastRadioPacketMs = millis();

  flightState = FlightState::POWER_ON;
  flashStatus(2, 100);
}

void loop() {
  uint32_t nowUs = micros();

  // IMU update
  if (nowUs - lastIMUUpdateUs >= (1000000UL / LOOP_IMU_HZ)) {
    float dt = (nowUs - lastIMUUpdateUs) / 1e6f;
    lastIMUUpdateUs = nowUs;
    updateAttitude(dt);
  }

  // Radio update
  if (nowUs - lastRadioUpdateUs >= (1000000UL / LOOP_RADIO_HZ)) {
    lastRadioUpdateUs = nowUs;

    RadioCommand cmd;
    if (receiveRadioCommand(cmd)) {
      currentCommand = cmd;
      digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED)); // Blink on packet
    } else if (radioTimeout()) {
      enterFailsafe();
    }
  }

  // State machine handling
  processStateMachine();

  // Control update
  if (nowUs - lastControlUpdateUs >= (1000000UL / LOOP_CONTROL_HZ)) {
    float dt = (nowUs - lastControlUpdateUs) / 1e6f;
    lastControlUpdateUs = nowUs;

    if (flightState == FlightState::ARMED) {
      updateMotorOutputs(dt);
    } else if (flightState == FlightState::MOTOR_SPIN) {
      // Already handled in state machine
    } else {
      stopMotors();
    }
  }

  // Telemetry update
  if (nowUs - lastTelemetryUs >= (1000000UL / LOOP_TELEMETRY_HZ)) {
    lastTelemetryUs = nowUs;
    if (linkEstablished) {
      sendTelemetry();
    }
  }
}
