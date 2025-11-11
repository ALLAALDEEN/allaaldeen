#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>
#include "MPU6050.h"  // Install from https://github.com/ElectronicCats/mpu6050 or i2cdevlib

// ----------------------------- Configuration ---------------------------------

// RF24 radio (CE, CSN)
static RF24 radio(7, 8);
static const uint64_t RADIO_PIPE = 0xE8E8F0F0E1LL;

// ESC pins (X-frame: front-right, rear-right, rear-left, front-left)
static const uint8_t ESC_PINS[4] = {3, 5, 9, 10};
static Servo esc[4];

// Status indicators
static const uint8_t BUZZER_PIN = 4;
static const uint8_t STATUS_LED_PIN = 6;

// MPU6050 I2C interrupt (optional)
static const uint8_t MPU_INT_PIN = 2;

// Loop timing
static const float LOOP_HZ = 500.0f;
static const unsigned long LOOP_PERIOD_US = (unsigned long)(1000000.0f / LOOP_HZ);

// Complementary filter constant
static const float COMPLEMENTARY_ALPHA = 0.98f;

// PID coefficients (tune for airframe)
static float pidRateKp[3] = {0.20f, 0.20f, 0.24f};
static float pidRateKi[3] = {0.02f, 0.02f, 0.04f};
static float pidRateKd[3] = {0.0025f, 0.0025f, 0.0030f};

static float pidAngleKp[3] = {4.5f, 4.5f, 3.5f};
static float pidAngleKi[3] = {0.02f, 0.02f, 0.0f};
static float pidAngleKd[3] = {0.20f, 0.20f, 0.0f};

// Throttle limits
static const uint16_t ESC_MIN_US = 1000;
static const uint16_t ESC_IDLE_US = 1120;
static const uint16_t ESC_MAX_US = 2000;
static const uint16_t ESC_SOFTSTART_TARGET = 1250;

// Safety thresholds
static const float MAX_TILT_DEG = 75.0f;
static const unsigned long RADIO_LOSS_US = 250000UL;  // 250 ms

// EEPROM calibration storage
static const uint32_t CAL_MAGIC = 0x53455256UL;  // 'SERV'

struct CalibrationData {
  uint32_t magic;
  float gyroBias[3];
  float accelBias[3];
};

static CalibrationData calData;

// ----------------------------- Radio protocol --------------------------------

enum RadioFlags : uint8_t {
  FLAG_ARM_REQUEST     = 0x01,
  FLAG_KILL_SWITCH     = 0x02,
  FLAG_CALIBRATE       = 0x04,
  FLAG_MOTOR_TEST      = 0x08,
  FLAG_ACK             = 0x10,
  FLAG_RESERVED1       = 0x20,
  FLAG_RESERVED2       = 0x40,
  FLAG_RESERVED3       = 0x80
};

struct __attribute__((packed)) RcPacket {
  uint8_t header;     // 0xA5
  uint8_t seq;
  uint8_t flags;
  uint16_t throttle;  // 1000-2000 us command
  int16_t roll;       // desired angle (centideg)
  int16_t pitch;
  int16_t yaw;        // desired rate (centideg/sec)
  uint16_t aux;       // button bitfield & UI ack
  uint8_t crc;
};

struct __attribute__((packed)) FcPacket {
  uint8_t header;     // 0x5A
  uint8_t seq;
  uint8_t status;     // bitfield: armed, calibrated, failsafe, imuOk, motorsOn
  uint16_t batteryMv;
  int16_t roll;       // actual angle (centideg)
  int16_t pitch;
  int16_t yawRate;    // deg/sec * 100
  uint16_t throttle;  // average motor output
  int16_t altitude;   // placeholder (cm)
  uint8_t crc;
};

enum FcStatusFlags : uint8_t {
  STATUS_ARMED     = 0x01,
  STATUS_CALIB     = 0x02,
  STATUS_FAILSAFE  = 0x04,
  STATUS_IMU_OK    = 0x08,
  STATUS_MOTORS    = 0x10,
  STATUS_RESERVED1 = 0x20,
  STATUS_RESERVED2 = 0x40,
  STATUS_RESERVED3 = 0x80
};

static RcPacket rcPacket = {};
static FcPacket fcPacket = {};

// ----------------------------- Flight states ---------------------------------

enum FlightState {
  STATE_DISARMED,
  STATE_ARMED_IDLE,
  STATE_ARMSOFTSTART,
  STATE_ARMED_ACTIVE,
  STATE_FAILSAFE
};

static FlightState flightState = STATE_DISARMED;
static bool imuHealthy = false;
static bool calInProgress = false;
static bool escSoftStarted = false;

// ----------------------------- IMU & PID -------------------------------------

static MPU6050 mpu;
static float angle[3] = {0.0f, 0.0f, 0.0f};     // roll, pitch, yaw (deg)
static float gyroRates[3] = {0.0f, 0.0f, 0.0f}; // deg/sec
static float rateIntegral[3] = {0.0f, 0.0f, 0.0f};
static float angleIntegral[3] = {0.0f, 0.0f, 0.0f};

static unsigned long lastLoopMicros = 0;
static unsigned long lastRadioMicros = 0;
static unsigned long lastTelemetryMicros = 0;

static uint8_t lastSeq = 0;
static bool radioLinked = false;

// ----------------------------- Utility ---------------------------------------

static uint8_t crc8_dallas(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    uint8_t inbyte = *data++;
    for (uint8_t i = 8; i; --i) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) {
        crc ^= 0x8C;
      }
      inbyte >>= 1;
    }
  }
  return crc;
}

static void flashStatus(uint8_t times, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < times; ++i) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(onMs);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(offMs);
  }
}

static void buzzerChirp(uint8_t chirps, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < chirps; ++i) {
    tone(BUZZER_PIN, 2200, onMs);
    delay(onMs + offMs);
  }
}

// ----------------------------- Calibration -----------------------------------

static void storeCalibration(const CalibrationData &data) {
  EEPROM.put(0, data);
}

static bool loadCalibration(CalibrationData &data) {
  EEPROM.get(0, data);
  return data.magic == CAL_MAGIC;
}

static void applyCalibration() {
  mpu.setXGyroOffset(calData.gyroBias[0]);
  mpu.setYGyroOffset(calData.gyroBias[1]);
  mpu.setZGyroOffset(calData.gyroBias[2]);
  // Accelerometer bias manually removed in code
}

static void calibrateIMU() {
  calInProgress = true;
  flashStatus(3, 100, 100);
  tone(BUZZER_PIN, 1600, 200);

  const uint16_t samples = 2000;
  float gyroSum[3] = {0};
  float accelSum[3] = {0};

  delay(100);
  for (uint16_t i = 0; i < samples; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gyroSum[0] += gx;
    gyroSum[1] += gy;
    gyroSum[2] += gz;
    accelSum[0] += ax;
    accelSum[1] += ay;
    accelSum[2] += az;
    delayMicroseconds(1000);
  }

  calData.magic = CAL_MAGIC;
  calData.gyroBias[0] = gyroSum[0] / samples;
  calData.gyroBias[1] = gyroSum[1] / samples;
  calData.gyroBias[2] = gyroSum[2] / samples;

  // Accelerometer offsets target 1G on Z
  const float accScale = 16384.0f; // LSB/g
  calData.accelBias[0] = accelSum[0] / samples;
  calData.accelBias[1] = accelSum[1] / samples;
  calData.accelBias[2] = (accelSum[2] / samples) - accScale;

  storeCalibration(calData);
  tone(BUZZER_PIN, 2200, 300);
  calInProgress = false;
}

static void calibrateESCs() {
  // Send max then min pulses to all ESCs
  for (uint8_t i = 0; i < 4; ++i) {
    esc[i].writeMicroseconds(ESC_MAX_US);
  }
  delay(3000);
  for (uint8_t i = 0; i < 4; ++i) {
    esc[i].writeMicroseconds(ESC_MIN_US);
  }
  delay(3000);
  escSoftStarted = false;
}

// ----------------------------- IMU update ------------------------------------

static void updateImu(float dt) {
  int16_t ax, ay, az, gx, gy, gz;
  if (!mpu.testConnection()) {
    imuHealthy = false;
    return;
  }

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Remove gyro bias from calibration
  gx -= calData.gyroBias[0];
  gy -= calData.gyroBias[1];
  gz -= calData.gyroBias[2];

  // Convert to deg/sec
  const float gyroScale = 131.0f;
  gyroRates[0] = gx / gyroScale;
  gyroRates[1] = gy / gyroScale;
  gyroRates[2] = gz / gyroScale;

  // Remove accel biases
  ax -= calData.accelBias[0];
  ay -= calData.accelBias[1];
  az -= calData.accelBias[2];

  float axf = ax / 16384.0f;
  float ayf = ay / 16384.0f;
  float azf = az / 16384.0f;

  float accelRoll = atan2f(ayf, azf) * RAD_TO_DEG;
  float accelPitch = atan2f(-axf, sqrtf(ayf * ayf + azf * azf)) * RAD_TO_DEG;

  angle[0] = COMPLEMENTARY_ALPHA * (angle[0] + gyroRates[0] * dt) +
             (1.0f - COMPLEMENTARY_ALPHA) * accelRoll;
  angle[1] = COMPLEMENTARY_ALPHA * (angle[1] + gyroRates[1] * dt) +
             (1.0f - COMPLEMENTARY_ALPHA) * accelPitch;
  angle[2] += gyroRates[2] * dt; // yaw integrates gyro only

  imuHealthy = true;
}

// ----------------------------- PID control -----------------------------------

static float constrainFloat(float x, float minVal, float maxVal) {
  if (x < minVal) return minVal;
  if (x > maxVal) return maxVal;
  return x;
}

static float runPid(float error, float rate, float dt, uint8_t axis) {
  // Angle PID outer loop -> rate setpoint
  float desiredRate = pidAngleKp[axis] * error +
                      pidAngleKi[axis] * angleIntegral[axis] -
                      pidAngleKd[axis] * rate;
  if (pidAngleKi[axis] > 0.0f) {
    angleIntegral[axis] += error * dt;
    angleIntegral[axis] = constrainFloat(angleIntegral[axis], -50.0f, 50.0f);
  }

  // Rate PID
  float rateError = desiredRate - rate;
  rateIntegral[axis] += rateError * dt;
  rateIntegral[axis] = constrainFloat(rateIntegral[axis], -200.0f, 200.0f);

  float output = pidRateKp[axis] * rateError +
                 pidRateKi[axis] * rateIntegral[axis] +
                 pidRateKd[axis] * (rateError / dt);
  return output;
}

// ----------------------------- Motor mixer -----------------------------------

static uint16_t motorCmd[4] = {ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US};
static uint16_t commandedThrottle = ESC_MIN_US;

static void updateMotors(float rollCmd, float pitchCmd, float yawCmd, uint16_t throttle) {
  float m1 = throttle + pitchCmd + yawCmd;  // Front-right
  float m2 = throttle - rollCmd - yawCmd;   // Rear-right
  float m3 = throttle - pitchCmd + yawCmd;  // Rear-left
  float m4 = throttle + rollCmd - yawCmd;   // Front-left

  motorCmd[0] = constrain((int)m1, ESC_MIN_US, ESC_MAX_US);
  motorCmd[1] = constrain((int)m2, ESC_MIN_US, ESC_MAX_US);
  motorCmd[2] = constrain((int)m3, ESC_MIN_US, ESC_MAX_US);
  motorCmd[3] = constrain((int)m4, ESC_MIN_US, ESC_MAX_US);
}

static void writeMotors() {
  for (uint8_t i = 0; i < 4; ++i) {
    esc[i].writeMicroseconds(motorCmd[i]);
  }
}

static void setAllMotors(uint16_t pulse) {
  for (uint8_t i = 0; i < 4; ++i) {
    motorCmd[i] = pulse;
    esc[i].writeMicroseconds(pulse);
  }
}

// ----------------------------- Radio handling --------------------------------

static void processRadioPacket() {
  RcPacket incoming;
  while (radio.available()) {
    radio.read(&incoming, sizeof(incoming));
  }

  if (incoming.header != 0xA5) {
    return;
  }
  uint8_t crc = crc8_dallas((uint8_t *)&incoming, sizeof(incoming) - 1);
  if (crc != incoming.crc) {
    return;
  }

  rcPacket = incoming;
  radioLinked = true;
  lastRadioMicros = micros();

  if (incoming.seq != lastSeq) {
    lastSeq = incoming.seq;
  }

  // Soft throttle floor: only allow <= ESC_IDLE_US while disarmed
  commandedThrottle = constrain(incoming.throttle, ESC_MIN_US, ESC_MAX_US);
  if (flightState == STATE_DISARMED || flightState == STATE_FAILSAFE) {
    if (commandedThrottle > ESC_IDLE_US) {
      commandedThrottle = ESC_IDLE_US;
    }
  } else {
    if (commandedThrottle < ESC_IDLE_US) {
      commandedThrottle = ESC_IDLE_US;
    }
  }

  // Handle calibration request
  if (incoming.flags & FLAG_CALIBRATE) {
    calibrateIMU();
    calibrateESCs();
  }
}

static void checkRadioLink() {
  unsigned long now = micros();
  if (radioLinked && (now - lastRadioMicros) > RADIO_LOSS_US) {
    radioLinked = false;
    flightState = STATE_FAILSAFE;
    buzzerChirp(3, 120, 120);
    setAllMotors(ESC_MIN_US);
  }
}

// ----------------------------- State machine ---------------------------------

static void updateStateMachine() {
  bool killActive = rcPacket.flags & FLAG_KILL_SWITCH;
  bool armRequest = rcPacket.flags & FLAG_ARM_REQUEST;
  bool motorTest = rcPacket.flags & FLAG_MOTOR_TEST;

  switch (flightState) {
    case STATE_DISARMED:
      setAllMotors(ESC_MIN_US);
      escSoftStarted = false;
      if (radioLinked && !killActive && armRequest &&
          commandedThrottle <= ESC_IDLE_US && fabs(rcPacket.yaw) < 50) {
        flightState = motorTest ? STATE_ARMSOFTSTART : STATE_ARMED_IDLE;
        buzzerChirp(1, 80, 80);
      }
      break;

    case STATE_ARMED_IDLE:
      setAllMotors(ESC_IDLE_US);
      if (killActive || !armRequest) {
        flightState = STATE_DISARMED;
        buzzerChirp(2, 60, 60);
      } else if (motorTest) {
        flightState = STATE_ARMSOFTSTART;
        escSoftStarted = false;
      } else if (commandedThrottle > ESC_IDLE_US + 10) {
        flightState = STATE_ARMED_ACTIVE;
        buzzerChirp(1, 100, 100);
      }
      break;

    case STATE_ARMSOFTSTART: {
      static unsigned long softStartBegin = 0;
      if (!escSoftStarted) {
        softStartBegin = millis();
        escSoftStarted = true;
      }
      unsigned long elapsed = millis() - softStartBegin;
      uint16_t target = ESC_IDLE_US;
      if (elapsed < 3000) {
        target = ESC_IDLE_US + (uint16_t)((ESC_SOFTSTART_TARGET - ESC_IDLE_US) * (elapsed / 3000.0f));
      } else {
        flightState = STATE_ARMED_ACTIVE;
      }
      setAllMotors(target);
      if (killActive || !armRequest) {
        flightState = STATE_DISARMED;
      }
      if (commandedThrottle > target + 20) {
        flightState = STATE_ARMED_ACTIVE;
      }
    } break;

    case STATE_ARMED_ACTIVE:
      if (killActive || !armRequest || commandedThrottle <= ESC_IDLE_US) {
        flightState = STATE_ARMED_IDLE;
        setAllMotors(ESC_IDLE_US);
      }
      if (!radioLinked) {
        flightState = STATE_FAILSAFE;
      }
      break;

    case STATE_FAILSAFE:
      setAllMotors(ESC_MIN_US);
      if (killActive && radioLinked) {
        flightState = STATE_DISARMED;
      }
      break;
  }
}

// ----------------------------- Telemetry -------------------------------------

static uint16_t readBatteryMv() {
  const uint8_t VBAT_PIN = A7;  // require voltage divider
  int raw = analogRead(VBAT_PIN);
  // Assuming 10-bit ADC, Vref = 5V, divider ratio 1:4 (R1=10k, R2=2.7k) -> factor 4.7
  const float VREF = 5.0f;
  const float RATIO = 4.7f;
  float voltage = (raw / 1023.0f) * VREF * RATIO;
  return (uint16_t)(voltage * 1000.0f);
}

static void sendTelemetry() {
  unsigned long now = micros();
  if (now - lastTelemetryMicros < 40000UL) {  // 25 Hz
    return;
  }
  lastTelemetryMicros = now;

  fcPacket.header = 0x5A;
  fcPacket.seq = rcPacket.seq;
  fcPacket.status = 0;
  if (flightState == STATE_ARMED_ACTIVE || flightState == STATE_ARMED_IDLE || flightState == STATE_ARMSOFTSTART) {
    fcPacket.status |= STATUS_ARMED;
  }
  if (calData.magic == CAL_MAGIC) {
    fcPacket.status |= STATUS_CALIB;
  }
  if (flightState == STATE_FAILSAFE) {
    fcPacket.status |= STATUS_FAILSAFE;
  }
  if (imuHealthy) {
    fcPacket.status |= STATUS_IMU_OK;
  }
  if (flightState == STATE_ARMED_ACTIVE || flightState == STATE_ARMSOFTSTART) {
    fcPacket.status |= STATUS_MOTORS;
  }

  fcPacket.batteryMv = readBatteryMv();
  fcPacket.roll = (int16_t)(angle[0] * 100.0f);
  fcPacket.pitch = (int16_t)(angle[1] * 100.0f);
  fcPacket.yawRate = (int16_t)(gyroRates[2] * 100.0f);
  uint32_t avgThrottle = motorCmd[0] + motorCmd[1] + motorCmd[2] + motorCmd[3];
  fcPacket.throttle = (uint16_t)(avgThrottle / 4);
  fcPacket.altitude = 0; // Placeholder

  fcPacket.crc = crc8_dallas((uint8_t *)&fcPacket, sizeof(fcPacket) - 1);

  radio.stopListening();
  radio.write(&fcPacket, sizeof(fcPacket));
  radio.startListening();
}

// ----------------------------- Setup & Loop ----------------------------------

static void initEscs() {
  for (uint8_t i = 0; i < 4; ++i) {
    esc[i].attach(ESC_PINS[i], 1000, 2000);
    esc[i].writeMicroseconds(ESC_MIN_US);
  }
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);

  Serial.begin(115200);
  Wire.begin();

  initEscs();

  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(108);
  radio.setRetries(5, 15);
  radio.openReadingPipe(1, RADIO_PIPE);
  radio.startListening();

  mpu.initialize();
  if (loadCalibration(calData)) {
    applyCalibration();
    flashStatus(2, 60, 60);
  } else {
    memset(&calData, 0, sizeof(calData));
    calibrateIMU();
    calibrateESCs();
  }

  lastLoopMicros = micros();
  lastRadioMicros = micros();

  buzzerChirp(2, 80, 80);
}

void loop() {
  unsigned long now = micros();
  if (now - lastLoopMicros < LOOP_PERIOD_US) {
    return;
  }
  float dt = (now - lastLoopMicros) / 1000000.0f;
  lastLoopMicros = now;

  if (radio.available()) {
    processRadioPacket();
  }
  checkRadioLink();

  updateStateMachine();

  if (imuHealthy || !calInProgress) {
    updateImu(dt);
  }

  if (flightState == STATE_ARMED_ACTIVE) {
    // Convert centidegree setpoints to degrees
    float targetRoll = rcPacket.roll / 100.0f;
    float targetPitch = rcPacket.pitch / 100.0f;
    float targetYawRate = rcPacket.yaw / 100.0f;

    float rollError = targetRoll - angle[0];
    float pitchError = targetPitch - angle[1];
    float yawRateError = targetYawRate - gyroRates[2];

    float rollCmd = runPid(rollError, gyroRates[0], dt, 0);
    float pitchCmd = runPid(pitchError, gyroRates[1], dt, 1);
    float yawCmd = pidRateKp[2] * yawRateError + pidRateKi[2] * rateIntegral[2] + pidRateKd[2] * (yawRateError / dt);

    if (fabs(angle[0]) > MAX_TILT_DEG || fabs(angle[1]) > MAX_TILT_DEG) {
      flightState = STATE_FAILSAFE;
      buzzerChirp(3, 100, 100);
    } else {
      updateMotors(rollCmd, pitchCmd, yawCmd, commandedThrottle);
    }
  } else if (flightState == STATE_ARMED_IDLE) {
    setAllMotors(ESC_IDLE_US);
  } else if (flightState == STATE_ARMSOFTSTART) {
    // Soft start handled in state machine
  } else if (flightState == STATE_FAILSAFE || flightState == STATE_DISARMED) {
    setAllMotors(ESC_MIN_US);
  }

  writeMotors();
  sendTelemetry();

  digitalWrite(STATUS_LED_PIN, (flightState == STATE_ARMED_ACTIVE) ? HIGH : LOW);
}
