/**
 * Quadcopter Flight Controller Firmware
 *
 * Hardware:
 *  - Arduino Nano
 *  - MPU6050 IMU
 *  - NRF24L01+ (CE -> D7, CSN -> D8)
 *  - 4x ESCs driving brushless motors (signal pins D3, D5, D6, D9)
 *  - Active buzzer (D4)
 *  - Status LED (D13)
 *
 * Features:
 *  - Complementary-filter attitude estimation
 *  - PID stabilization for roll/pitch/yaw
 *  - NRF24 telemetry/command link with transmitter ack payloads
 *  - IMU & ESC calibration stored in EEPROM
 *  - Guided arming workflow with safety checks
 *  - Smooth spool-up demo mode before flight
 *  - Connection watchdog & failsafe motor cut
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <EEPROM.h>
#include <math.h>

// -------------------- Pin Mapping --------------------
static const uint8_t ESC_PINS[4] = {3, 5, 6, 9};
static const uint8_t BUZZER_PIN = 4;
static const uint8_t STATUS_LED_PIN = 13;
static const uint8_t NRF_CE_PIN = 7;
static const uint8_t NRF_CSN_PIN = 8;

// -------------------- Radio --------------------------
static const byte RADIO_RX_PIPE[6] = "FCCTL";
static const byte RADIO_TX_PIPE[6] = "TXCTL";
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// -------------------- MPU6050 ------------------------
static const uint8_t MPU_ADDR = 0x68;
static const float ACCEL_SCALE = 16384.0f;    // LSB/g for +/-2g
static const float GYRO_SCALE = 131.0f;       // LSB/(deg/s) for +/-250 dps

// -------------------- Timing -------------------------
static const uint16_t LOOP_PERIOD_US = 4000;  // 250 Hz loop
static const uint32_t COMMAND_TIMEOUT_US = 500000;  // 0.5 s watchdog

// -------------------- Flight Dynamics ----------------
static const float COMPLEMENTARY_ALPHA = 0.98f;
static const float MAX_TILT_DEG = 35.0f;
static const float MAX_YAW_RATE_DPS = 180.0f;
static const uint16_t THROTTLE_MIN = 1000;
static const uint16_t THROTTLE_MAX = 2000;
static const uint16_t THROTTLE_ARM_THRESHOLD = 1100;
static const uint16_t THROTTLE_SPOOL_TARGET = 1150;
static const uint8_t SPOOL_RAMP_STEP = 2;  // microseconds per loop during spool

// PID tuning (basic defaults, tune per airframe)
static const float PID_ROLL_KP = 3.5f;
static const float PID_ROLL_KI = 0.02f;
static const float PID_ROLL_KD = 0.18f;

static const float PID_PITCH_KP = 3.5f;
static const float PID_PITCH_KI = 0.02f;
static const float PID_PITCH_KD = 0.18f;

static const float PID_YAW_KP = 2.0f;
static const float PID_YAW_KI = 0.01f;
static const float PID_YAW_KD = 0.0f;

// -------------------- Data Structures ----------------
enum GuideStep : uint8_t {
  GUIDE_HANDSHAKE = 0,
  GUIDE_KILL_SWITCH_CHECK = 1,
  GUIDE_CALIBRATE_CONFIRM = 2,
  GUIDE_ARM_REQUEST = 3,
  GUIDE_SPOOL_REQUEST = 4,
  GUIDE_READY = 5
};

struct RcCommand {
  uint16_t throttle;   // 1000 - 2000
  int16_t roll;        // 1000 - 2000 centered at 1500
  int16_t pitch;       // 1000 - 2000 centered at 1500
  int16_t yaw;         // 1000 - 2000 centered at 1500
  uint8_t switches;    // bit0: kill switch (1 = kill), bit1: unused
  uint8_t buttons;     // bit0: button1 (calibrate), bit1: button2 (spool)
  uint8_t guideStep;   // desired workflow step
  uint8_t reserved;    // align to 2-byte boundary
  uint32_t sequence;   // rolling counter
} __attribute__((packed));

struct TelemetryPacket {
  uint16_t throttleEcho;
  int16_t rollDegX10;
  int16_t pitchDegX10;
  int16_t yawDegX10;
  float estimatedAltitudeMeters;
  uint8_t statusFlags;   // bit0: link, bit1: armed, bit2: calibrating, bit3: spool, bit4: calibValid
  uint8_t guideStepAck;  // current state machine step
  uint16_t loopMicros;
  uint16_t reserved;
} __attribute__((packed));

struct CalibrationData {
  uint32_t magic;
  float gyroOffsets[3];
  float accelOffsets[3];
};

// -------------------- Globals ------------------------
Servo esc[4];
RcCommand lastCommand = {};
TelemetryPacket telemetry = {};
CalibrationData calib = {};

float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float yawDeg = 0.0f;
float verticalVelocity = 0.0f;
float estimatedAltitude = 0.0f;

float integralRoll = 0.0f;
float integralPitch = 0.0f;
float integralYaw = 0.0f;

uint8_t lastButtons = 0;
uint8_t lastSwitches = 0;
bool linkActive = false;
bool calibrationValid = false;
bool isCalibrating = false;

enum FlightState : uint8_t {
  STATE_DISARMED = 0,
  STATE_ARMED = 1,
  STATE_SPOOL = 2
};
FlightState flightState = STATE_DISARMED;
GuideStep guideState = GUIDE_HANDSHAKE;

uint16_t spoolThrottle = THROTTLE_MIN;
uint32_t lastCommandMicros = 0;
uint32_t lastLoopMicros = 0;
bool statusLedState = false;

// -------------------- Forward Declarations -----------
void setupRadio();
void setupMpu();
void loadCalibration();
void saveCalibration();
void calibrateImu();
void calibrateEscs();
bool receiveCommand();
void updateTelemetry(uint16_t loopTime);
void applyFailsafe();
void updateFlightControl(float dtSec);
void writeMotors(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
void setAllMotors(uint16_t value);
void updateStatusLed();
void beep(uint16_t frequency, uint16_t durationMs);

// -------------------- Utility Helpers ----------------
static float constrainFloat(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

static float normalizeChannel(int16_t pulseWidth) {
  // 1000 -> -1.0, 1500 -> 0.0, 2000 -> +1.0
  return constrainFloat((pulseWidth - 1500) / 500.0f, -1.0f, 1.0f);
}

// -------------------- Setup & Loop -------------------
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  for (uint8_t i = 0; i < 4; ++i) {
    esc[i].attach(ESC_PINS[i], THROTTLE_MIN, THROTTLE_MAX);
  }
  setAllMotors(THROTTLE_MIN);

  Serial.begin(115200);
  delay(100);
  Serial.println(F("Flight Controller Booting..."));

  Wire.begin();
  setupMpu();
  loadCalibration();

  setupRadio();

  lastLoopMicros = micros();
  lastCommandMicros = micros();
}

void loop() {
  const uint32_t loopStart = micros();
  const uint16_t loopTime = loopStart - lastLoopMicros;
  lastLoopMicros = loopStart;

  if (receiveCommand()) {
    lastCommandMicros = loopStart;
    if (!linkActive) {
      linkActive = true;
      beep(2800, 180);
    }
    updateStatusLed();
  } else {
    if (linkActive && (loopStart - lastCommandMicros) > COMMAND_TIMEOUT_US) {
      linkActive = false;
      flightState = STATE_DISARMED;
      setAllMotors(THROTTLE_MIN);
      beep(1800, 250);
    }
  }

  applyFailsafe();

  const float dt = loopTime / 1000000.0f;
  updateFlightControl(dt);
  updateTelemetry(loopTime);

  const uint32_t loopDuration = micros() - loopStart;
  if (loopDuration < LOOP_PERIOD_US) {
    delayMicroseconds(LOOP_PERIOD_US - loopDuration);
  }
}

// -------------------- Radio Handling -----------------
void setupRadio() {
  if (!radio.begin()) {
    Serial.println(F("RF24 init failed"));
    while (true) {
      digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
      delay(200);
    }
  }
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(81);
  radio.setCRCLength(RF24_CRC_16);
  radio.setRetries(5, 15);
  radio.openReadingPipe(1, RADIO_RX_PIPE);
  radio.openWritingPipe(RADIO_TX_PIPE);
  radio.enableAckPayload();
  radio.startListening();
  Serial.println(F("RF24 ready"));
}

bool receiveCommand() {
  bool newData = false;
  while (radio.available()) {
    radio.read(&lastCommand, sizeof(lastCommand));
    newData = true;
  }
  return newData;
}

void updateTelemetry(uint16_t loopTime) {
  telemetry.throttleEcho = lastCommand.throttle;
  telemetry.rollDegX10 = static_cast<int16_t>(rollDeg * 10.0f);
  telemetry.pitchDegX10 = static_cast<int16_t>(pitchDeg * 10.0f);
  telemetry.yawDegX10 = static_cast<int16_t>(yawDeg * 10.0f);
  telemetry.estimatedAltitudeMeters = estimatedAltitude;
  telemetry.statusFlags = 0;
  if (linkActive) telemetry.statusFlags |= (1 << 0);
  if (flightState == STATE_ARMED) telemetry.statusFlags |= (1 << 1);
  if (isCalibrating) telemetry.statusFlags |= (1 << 2);
  if (flightState == STATE_SPOOL) telemetry.statusFlags |= (1 << 3);
  if (calibrationValid) telemetry.statusFlags |= (1 << 4);
  telemetry.guideStepAck = static_cast<uint8_t>(guideState);
  telemetry.loopMicros = loopTime;

  // Push latest telemetry as ack payload so transmitter reads it on next exchange
  radio.writeAckPayload(1, &telemetry, sizeof(telemetry));
}

// -------------------- Safety & State -----------------
void applyFailsafe() {
  const bool killSwitch = (lastCommand.switches & 0x01) != 0;
  const bool buttonCalibrate = (lastCommand.buttons & 0x01) != 0;
  const bool buttonSpool = (lastCommand.buttons & 0x02) != 0;

  // Handle guided workflow state machine
  if (!linkActive) {
    guideState = GUIDE_HANDSHAKE;
  } else {
    switch (guideState) {
      case GUIDE_HANDSHAKE:
        guideState = killSwitch ? GUIDE_KILL_SWITCH_CHECK : GUIDE_HANDSHAKE;
        break;
      case GUIDE_KILL_SWITCH_CHECK:
        if (!killSwitch) {
          guideState = GUIDE_CALIBRATE_CONFIRM;
        }
        break;
      case GUIDE_CALIBRATE_CONFIRM:
        if (buttonCalibrate && !(lastButtons & 0x01)) {
          isCalibrating = true;
          calibrationValid = false;
          calibrateImu();
          calibrateEscs();
          isCalibrating = false;
          calibrationValid = true;
          rollDeg = pitchDeg = yawDeg = 0.0f;
          verticalVelocity = 0.0f;
          estimatedAltitude = 0.0f;
          integralRoll = integralPitch = integralYaw = 0.0f;
          beep(3400, 120);
          delay(80);
          beep(3600, 120);
          guideState = GUIDE_ARM_REQUEST;
        }
        break;
      case GUIDE_ARM_REQUEST:
        if (!killSwitch && calibrationValid &&
            lastCommand.throttle < THROTTLE_ARM_THRESHOLD &&
            lastCommand.guideStep == GUIDE_ARM_REQUEST) {
          flightState = STATE_ARMED;
          guideState = GUIDE_SPOOL_REQUEST;
        }
        break;
      case GUIDE_SPOOL_REQUEST:
        if (buttonSpool && !(lastButtons & 0x02) && flightState == STATE_ARMED) {
          flightState = STATE_SPOOL;
          spoolThrottle = THROTTLE_MIN;
        }
        if (flightState == STATE_SPOOL && lastCommand.guideStep == GUIDE_READY) {
          flightState = STATE_ARMED;
          guideState = GUIDE_READY;
        }
        break;
      case GUIDE_READY:
      default:
        break;
    }
  }

  // Kill switch always overrides
  if (killSwitch || !linkActive) {
    flightState = STATE_DISARMED;
    spoolThrottle = THROTTLE_MIN;
  }

  // Edge detect buttons for logging
  if ((buttonCalibrate) && !(lastButtons & 0x01)) {
    Serial.println(F("Calibration requested"));
  }
  if ((buttonSpool) && !(lastButtons & 0x02)) {
    Serial.println(F("Smooth spool requested"));
  }
  lastButtons = lastCommand.buttons;
  lastSwitches = lastCommand.switches;
}

void updateFlightControl(float dt) {
  // Read IMU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, static_cast<uint8_t>(14));
  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temperature discard
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  const float ax = (rawAx - calib.accelOffsets[0]) / ACCEL_SCALE;
  const float ay = (rawAy - calib.accelOffsets[1]) / ACCEL_SCALE;
  const float az = (rawAz - calib.accelOffsets[2]) / ACCEL_SCALE;
  const float gx = (rawGx - calib.gyroOffsets[0]) / GYRO_SCALE;
  const float gy = (rawGy - calib.gyroOffsets[1]) / GYRO_SCALE;
  const float gz = (rawGz - calib.gyroOffsets[2]) / GYRO_SCALE;

  const float accelRoll = atan2f(ay, az) * RAD_TO_DEG;
  const float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  rollDeg = COMPLEMENTARY_ALPHA * (rollDeg + gx * dt) + (1.0f - COMPLEMENTARY_ALPHA) * accelRoll;
  pitchDeg = COMPLEMENTARY_ALPHA * (pitchDeg + gy * dt) + (1.0f - COMPLEMENTARY_ALPHA) * accelPitch;
  yawDeg += gz * dt;  // integrate yaw, wrap later if desired
  if (yawDeg > 180.0f) yawDeg -= 360.0f;
  if (yawDeg < -180.0f) yawDeg += 360.0f;

  // crude vertical estimation (will drift, for display only)
  const float accelZCorrected = (az - 1.0f);  // subtract gravity (~1g)
  verticalVelocity += accelZCorrected * 9.81f * dt;
  verticalVelocity = constrainFloat(verticalVelocity * 0.98f, -5.0f, 5.0f); // dampening
  estimatedAltitude += verticalVelocity * dt;
  estimatedAltitude = constrainFloat(estimatedAltitude, -5.0f, 100.0f);

  if (flightState == STATE_DISARMED) {
    integralRoll = integralPitch = integralYaw = 0.0f;
    setAllMotors(THROTTLE_MIN);
    return;
  }

  const float targetRoll = normalizeChannel(lastCommand.roll) * MAX_TILT_DEG;
  const float targetPitch = normalizeChannel(lastCommand.pitch) * MAX_TILT_DEG;
  const float targetYawRate = normalizeChannel(lastCommand.yaw) * MAX_YAW_RATE_DPS;

  const float rollError = targetRoll - rollDeg;
  const float pitchError = targetPitch - pitchDeg;
  const float yawRate = gz;
  const float yawError = targetYawRate - yawRate;

  integralRoll += rollError * dt;
  integralPitch += pitchError * dt;
  integralYaw += yawError * dt;

  integralRoll = constrainFloat(integralRoll, -100.0f, 100.0f);
  integralPitch = constrainFloat(integralPitch, -100.0f, 100.0f);
  integralYaw = constrainFloat(integralYaw, -50.0f, 50.0f);

  const float derivativeRoll = gx;
  const float derivativePitch = gy;
  const float derivativeYaw = gz; // yaw derivative from gyro rate

  const float rollOutput = PID_ROLL_KP * rollError +
                           PID_ROLL_KI * integralRoll -
                           PID_ROLL_KD * derivativeRoll;
  const float pitchOutput = PID_PITCH_KP * pitchError +
                            PID_PITCH_KI * integralPitch -
                            PID_PITCH_KD * derivativePitch;
  const float yawOutput = PID_YAW_KP * yawError +
                          PID_YAW_KI * integralYaw -
                          PID_YAW_KD * derivativeYaw;

  uint16_t commandedThrottle = lastCommand.throttle;

  if (flightState == STATE_SPOOL) {
    uint16_t nextSpool = spoolThrottle + SPOOL_RAMP_STEP;
    if (nextSpool > THROTTLE_SPOOL_TARGET) {
      nextSpool = THROTTLE_SPOOL_TARGET;
    }
    spoolThrottle = nextSpool;
    commandedThrottle = spoolThrottle;
  }

  commandedThrottle = constrain(commandedThrottle, THROTTLE_MIN, THROTTLE_MAX);

  float mFrontLeft = commandedThrottle - pitchOutput - rollOutput - yawOutput;
  float mFrontRight = commandedThrottle - pitchOutput + rollOutput + yawOutput;
  float mRearRight = commandedThrottle + pitchOutput + rollOutput - yawOutput;
  float mRearLeft = commandedThrottle + pitchOutput - rollOutput + yawOutput;

  mFrontLeft = constrainFloat(mFrontLeft, THROTTLE_MIN, THROTTLE_MAX);
  mFrontRight = constrainFloat(mFrontRight, THROTTLE_MIN, THROTTLE_MAX);
  mRearRight = constrainFloat(mRearRight, THROTTLE_MIN, THROTTLE_MAX);
  mRearLeft = constrainFloat(mRearLeft, THROTTLE_MIN, THROTTLE_MAX);

  writeMotors(static_cast<uint16_t>(mFrontLeft),
              static_cast<uint16_t>(mFrontRight),
              static_cast<uint16_t>(mRearRight),
              static_cast<uint16_t>(mRearLeft));
}

void writeMotors(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
  esc[0].writeMicroseconds(m1);
  esc[1].writeMicroseconds(m2);
  esc[2].writeMicroseconds(m3);
  esc[3].writeMicroseconds(m4);
}

void setAllMotors(uint16_t value) {
  for (uint8_t i = 0; i < 4; ++i) {
    esc[i].writeMicroseconds(value);
  }
}

void updateStatusLed() {
  statusLedState = !statusLedState;
  digitalWrite(STATUS_LED_PIN, statusLedState);
}

void beep(uint16_t frequency, uint16_t durationMs) {
  tone(BUZZER_PIN, frequency, durationMs);
}

// -------------------- Calibration --------------------
void setupMpu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG: +/-250dps
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG: +/-2g
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(100);
}

void loadCalibration() {
  EEPROM.get(0, calib);
  if (calib.magic != 0xA5A5BEEF) {
    Serial.println(F("No valid calibration found"));
    memset(&calib, 0, sizeof(calib));
    calib.magic = 0xA5A5BEEF;
    calibrationValid = false;
  } else {
    calibrationValid = true;
    Serial.println(F("Calibration loaded"));
  }
}

void saveCalibration() {
  EEPROM.put(0, calib);
}

void calibrateImu() {
    Serial.println(F("Starting IMU calibration... Keep drone still."));
  const uint16_t samples = 2000;
  float sumG[3] = {0, 0, 0};
  float sumA[3] = {0, 0, 0};

  for (uint16_t i = 0; i < samples; ++i) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, static_cast<uint8_t>(14));
    int16_t rawAx = (Wire.read() << 8) | Wire.read();
    int16_t rawAy = (Wire.read() << 8) | Wire.read();
    int16_t rawAz = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
    int16_t rawGx = (Wire.read() << 8) | Wire.read();
    int16_t rawGy = (Wire.read() << 8) | Wire.read();
    int16_t rawGz = (Wire.read() << 8) | Wire.read();

    sumA[0] += rawAx;
    sumA[1] += rawAy;
    sumA[2] += rawAz;
    sumG[0] += rawGx;
    sumG[1] += rawGy;
    sumG[2] += rawGz;
    delay(2);
  }

  calib.gyroOffsets[0] = sumG[0] / samples;
  calib.gyroOffsets[1] = sumG[1] / samples;
  calib.gyroOffsets[2] = sumG[2] / samples;

  calib.accelOffsets[0] = sumA[0] / samples;
  calib.accelOffsets[1] = sumA[1] / samples;
  calib.accelOffsets[2] = (sumA[2] / samples) - ACCEL_SCALE; // remove gravity

  saveCalibration();
  Serial.println(F("IMU calibration complete"));
}

void calibrateEscs() {
  Serial.println(F("ESC calibration - ensure props removed!"));
  setAllMotors(THROTTLE_MAX);
  delay(2000);
  setAllMotors(THROTTLE_MIN);
  delay(3000);
  Serial.println(F("ESC calibration done"));
}
