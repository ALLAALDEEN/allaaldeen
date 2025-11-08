/**
 * DIY Quadcopter Flight Controller
 *
 * Hardware:
 *  - Arduino Nano (ATmega328P)
 *  - nRF24L01+ (CE=D4, CSN=D10)
 *  - MPU6050 IMU (INT=D2)
 *  - GY-36 MS5611 barometer (I2C)
 *  - ESC / Motors: FL=D3, FR=D5, RR=D6, RL=D7
 *  - Buzzer: D8
 *  - Status LED: D13 (recommended, original plan used D7 which conflicts with RL motor)
 *
 * Author: GPT-5 Codex
 */

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <math.h>
#include <RF24.h>

#include "../shared/rc_protocol.h"

// ---------------------- Pin Assignments ----------------------
static constexpr uint8_t PIN_NRF_CE = 4;
static constexpr uint8_t PIN_NRF_CSN = 10;
static constexpr uint8_t PIN_IMU_INT = 2;

static constexpr uint8_t PIN_MOTOR_FL = 3;
static constexpr uint8_t PIN_MOTOR_FR = 5;
static constexpr uint8_t PIN_MOTOR_RR = 6;
static constexpr uint8_t PIN_MOTOR_RL = 7;

static constexpr uint8_t PIN_BUZZER = 8;
static constexpr uint8_t PIN_STATUS_LED = LED_BUILTIN; // Prefer D13 to avoid motor conflict.

// ---------------------- Radio Configuration ----------------------
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
static constexpr uint8_t kRadioPipeController = 1;
static constexpr uint8_t kRadioPipeTelemetry = 2;
const uint8_t kControllerAddress[6] = {'R', 'C', 'L', 'N', 'K'};
const uint8_t kTelemetryAddress[6] = {'D', 'R', 'O', 'N', 'E'};

// RC button bits (must match transmitter sketch)
static constexpr uint16_t BUTTON_ARM = 0x0001;
static constexpr uint16_t BUTTON_DISARM = 0x0002;
static constexpr uint16_t BUTTON_BEEP = 0x0004;

// RC switch bits
static constexpr uint16_t SWITCH_MODE = 0x0001;    // SW_1
static constexpr uint16_t SWITCH_ALT_HOLD = 0x0002; // SW_2

// ---------------------- Timing ----------------------
static constexpr uint32_t LOOP_PERIOD_US = 4000; // 250 Hz control loop
static constexpr uint32_t SIGNAL_TIMEOUT_MS = 500; // Fail-safe timeout

// ---------------------- Motor Control ----------------------
Servo motorFL, motorFR, motorRR, motorRL;
static constexpr uint16_t MOTOR_MIN_US = 1000;
static constexpr uint16_t MOTOR_MAX_US = 2000;
static constexpr uint16_t MOTOR_ARM_US = 1100;

// ---------------------- IMU Driver ----------------------
namespace imu {

static constexpr uint8_t kAddress = 0x68;
static constexpr uint8_t kRegisterPwrMgmt1 = 0x6B;
static constexpr uint8_t kRegisterConfig = 0x1A;
static constexpr uint8_t kRegisterGyroConfig = 0x1B;
static constexpr uint8_t kRegisterAccelConfig = 0x1C;
static constexpr uint8_t kRegisterAccel = 0x3B;
static constexpr uint8_t kRegisterTemp = 0x41;
static constexpr uint8_t kRegisterGyro = 0x43;

struct RawReadings {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

struct Calibration {
  float gyro_offset_x = 0;
  float gyro_offset_y = 0;
  float gyro_offset_z = 0;
  float accel_offset_x = 0;
  float accel_offset_y = 0;
  float accel_offset_z = 0;
};

Calibration calibration;

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kAddress);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readRegisters(uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(kAddress);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(kAddress, count);
  for (uint8_t i = 0; i < count && Wire.available(); ++i) {
    dest[i] = Wire.read();
  }
}

bool begin() {
  writeRegister(kRegisterPwrMgmt1, 0x00); // Wake up
  delay(100);
  writeRegister(kRegisterConfig, 0x03);      // DLPF 44Hz
  writeRegister(kRegisterGyroConfig, 0x08);  // ±500 dps
  writeRegister(kRegisterAccelConfig, 0x10); // ±8g
  return true;
}

RawReadings readRaw() {
  uint8_t buffer[14];
  readRegisters(kRegisterAccel, 14, buffer);
  RawReadings out;
  out.ax = (int16_t)((buffer[0] << 8) | buffer[1]);
  out.ay = (int16_t)((buffer[2] << 8) | buffer[3]);
  out.az = (int16_t)((buffer[4] << 8) | buffer[5]);
  out.gx = (int16_t)((buffer[8] << 8) | buffer[9]);
  out.gy = (int16_t)((buffer[10] << 8) | buffer[11]);
  out.gz = (int16_t)((buffer[12] << 8) | buffer[13]);
  return out;
}

void calibrate(uint16_t samples = 1000) {
  calibration = Calibration();
  const float alpha = 1.0f / samples;
  for (uint16_t i = 0; i < samples; ++i) {
    RawReadings raw = readRaw();
    calibration.gyro_offset_x += raw.gx * alpha;
    calibration.gyro_offset_y += raw.gy * alpha;
    calibration.gyro_offset_z += raw.gz * alpha;
    calibration.accel_offset_x += raw.ax * alpha;
    calibration.accel_offset_y += raw.ay * alpha;
    calibration.accel_offset_z += raw.az * alpha;
    delay(2);
  }
  // Expected gravity on Z axis for ±8g: 1g = 4096 LSB
  calibration.accel_offset_z -= 4096.0f;
}

} // namespace imu

struct AttitudeState {
  float roll_deg = 0.0f;
  float pitch_deg = 0.0f;
  float yaw_rate_dps = 0.0f;
};

AttitudeState attitude;

void updateAttitude(float dt_seconds) {
  imu::RawReadings raw = imu::readRaw();

  const float gyroScale = 65.5f;  // 500dps -> 65.5 LSB/deg/s
  const float accelScale = 4096.0f; // ±8g -> 4096 LSB/g

  float gx = (raw.gx - imu::calibration.gyro_offset_x) / gyroScale;
  float gy = (raw.gy - imu::calibration.gyro_offset_y) / gyroScale;
  float gz = (raw.gz - imu::calibration.gyro_offset_z) / gyroScale;

  float ax = (raw.ax - imu::calibration.accel_offset_x) / accelScale;
  float ay = (raw.ay - imu::calibration.accel_offset_y) / accelScale;
  float az = (raw.az - imu::calibration.accel_offset_z) / accelScale;

  // Gyro integration
  attitude.roll_deg += gx * dt_seconds;
  attitude.pitch_deg += gy * dt_seconds;
  attitude.yaw_rate_dps = gz;

  // Accel angle (degrees)
  float accel_roll = atan2f(ay, az) * RAD_TO_DEG;
  float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  const float alpha = 0.04f; // complementary filter coefficient
  attitude.roll_deg = (1.0f - alpha) * attitude.roll_deg + alpha * accel_roll;
  attitude.pitch_deg = (1.0f - alpha) * attitude.pitch_deg + alpha * accel_pitch;
}

// ---------------------- MS5611 Driver ----------------------
namespace ms5611 {

static constexpr uint8_t kAddress = 0x77;

enum class Oversampling : uint8_t {
  OSR256 = 0x00,
  OSR512 = 0x02,
  OSR1024 = 0x04,
  OSR2048 = 0x06,
  OSR4096 = 0x08,
};

uint16_t C[7] = {0};
float referencePressure = 101325.0f;
uint32_t lastConversionStart = 0;
bool readingTemperature = true;

void reset() {
  Wire.beginTransmission(kAddress);
  Wire.write(0x1E);
  Wire.endTransmission();
  delay(5);
}

uint16_t readPROM(uint8_t index) {
  Wire.beginTransmission(kAddress);
  Wire.write(0xA0 + index * 2);
  Wire.endTransmission();
  Wire.requestFrom(kAddress, (uint8_t)2);
  uint16_t value = (Wire.read() << 8) | Wire.read();
  return value;
}

void begin() {
  reset();
  for (uint8_t i = 0; i < 6; ++i) {
    C[i + 1] = readPROM(i + 1);
  }
  startConversion(0x40 | static_cast<uint8_t>(Oversampling::OSR4096));
}

void startConversion(uint8_t command) {
  Wire.beginTransmission(kAddress);
  Wire.write(command);
  Wire.endTransmission();
  lastConversionStart = micros();
}

uint32_t readADC() {
  Wire.beginTransmission(kAddress);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(kAddress, (uint8_t)3);
  uint32_t value = ((uint32_t)Wire.read() << 16) |
                   ((uint32_t)Wire.read() << 8) |
                   (uint32_t)Wire.read();
  return value;
}

float pressurePa = 101325.0f;
float temperatureC = 25.0f;

void update() {
  const uint32_t conversionTimeUs = 9100; // for OSR4096
  if (micros() - lastConversionStart < conversionTimeUs) {
    return;
  }

  static uint32_t D1 = 0; // Pressure raw
  static uint32_t D2 = 0; // Temperature raw

  if (readingTemperature) {
    D2 = readADC();
    startConversion(0x40 | static_cast<uint8_t>(Oversampling::OSR4096));
  } else {
    D1 = readADC();
    startConversion(0x50 | static_cast<uint8_t>(Oversampling::OSR4096));
  }
  readingTemperature = !readingTemperature;

  if (!readingTemperature) {
    return; // wait until both conversions available
  }

  int32_t dT = D2 - ((uint32_t)C[5] << 8);
  int64_t OFF = ((int64_t)C[2] << 16) + (((int64_t)dT * C[4]) >> 7);
  int64_t SENS = ((int64_t)C[1] << 15) + (((int64_t)dT * C[3]) >> 8);

  int32_t TEMP = 2000 + ((int64_t)dT * C[6]) / (1 << 23);

  if (TEMP < 2000) {
    int32_t t2 = (dT * dT) >> 31;
    int32_t delta = TEMP - 2000;
    int64_t off2 = (5 * (int64_t)delta * (int64_t)delta) >> 1;
    int64_t sens2 = off2 >> 1;
    if (TEMP < -1500) {
      int32_t delta2 = TEMP + 1500;
      off2 += 7 * (int64_t)delta2 * (int64_t)delta2;
      sens2 += (11 * (int64_t)delta2 * (int64_t)delta2) >> 1;
    }
    TEMP -= t2;
    OFF -= off2;
    SENS -= sens2;
  }

  int32_t pressure = (((int64_t)D1 * SENS) >> 21) - OFF;
  pressure = pressure >> 15;

  pressurePa = (float)pressure;
  temperatureC = TEMP / 100.0f;
}

float altitudeMeters() {
  // International Standard Atmosphere
  return 44330.0f * (1.0f - powf(pressurePa / referencePressure, 0.1903f));
}

void calibrate(float baselinePa) {
  referencePressure = baselinePa;
}

float getPressurePa() { return pressurePa; }
float getTemperatureC() { return temperatureC / 100.0f; }

} // namespace ms5611

// ---------------------- PID Controller ----------------------
struct PID {
  float kp, ki, kd;
  float integrator = 0.0f;
  float prevError = 0.0f;
  float outMin, outMax;

  float compute(float target, float measurement, float dt) {
    float error = target - measurement;
    integrator += error * ki * dt;
    integrator = constrain(integrator, outMin, outMax);
    float derivative = (error - prevError) / dt;
    float output = kp * error + integrator + kd * derivative;
    output = constrain(output, outMin, outMax);
    prevError = error;
    return output;
  }

  void reset() {
    integrator = 0.0f;
    prevError = 0.0f;
  }
};

PID pidRoll{4.0f, 1.8f, 0.08f, -200.0f, 200.0f};
PID pidPitch{4.0f, 1.8f, 0.08f, -200.0f, 200.0f};
PID pidYaw{2.5f, 0.6f, 0.0f, -150.0f, 150.0f};

// ---------------------- Control State ----------------------
rc::ControlPacket controlPacket;
bool radioHasSignal = false;
bool isArmed = false;
uint32_t lastSignalMs = 0;

float altitudeTargetM = 0.0f;
bool altitudeHold = false;

void setAllMotors(uint16_t microseconds) {
  motorFL.writeMicroseconds(microseconds);
  motorFR.writeMicroseconds(microseconds);
  motorRR.writeMicroseconds(microseconds);
  motorRL.writeMicroseconds(microseconds);
}

void signalArming(bool armed) {
  digitalWrite(PIN_STATUS_LED, armed ? HIGH : LOW);
  tone(PIN_BUZZER, armed ? 2000 : 1200, 100);
  delay(150);
  noTone(PIN_BUZZER);
}

void setupRadio() {
  radio.begin();
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setRetries(5, 15);
  radio.setChannel(90);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openReadingPipe(kRadioPipeController, kControllerAddress);
  radio.openWritingPipe(kTelemetryAddress);
  radio.startListening();
}

void setupMotors() {
  motorFL.attach(PIN_MOTOR_FL, 900, 2100);
  motorFR.attach(PIN_MOTOR_FR, 900, 2100);
  motorRR.attach(PIN_MOTOR_RR, 900, 2100);
  motorRL.attach(PIN_MOTOR_RL, 900, 2100);
  setAllMotors(MOTOR_MIN_US);
}

void calibrateSensors() {
  Serial.println(F("Calibrating IMU... Keep the frame steady."));
  imu::calibrate(800);
  Serial.println(F("IMU calibration complete."));

  Serial.println(F("Calibrating barometer baseline..."));
  float sum = 0.0f;
  for (int i = 0; i < 100; ++i) {
    ms5611::update();
    sum += ms5611::getPressurePa();
    delay(10);
  }
  ms5611::calibrate(sum / 100.0f);
  altitudeTargetM = 0.0f;
  Serial.println(F("Barometer baseline set."));
}

void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
  noTone(PIN_BUZZER);

  Serial.begin(115200);
  delay(100);
  Serial.println(F("Booting flight controller..."));

  Wire.begin();
  Wire.setClock(400000);

  imu::begin();
  ms5611::begin();
  setupMotors();
  setupRadio();
  calibrateSensors();

  tone(PIN_BUZZER, 1600, 120);
  delay(150);
  tone(PIN_BUZZER, 2000, 120);
  delay(150);
  noTone(PIN_BUZZER);
  Serial.println(F("Flight controller ready."));
}

bool shouldArm(const rc::ControlPacket& packet) {
  const bool armRequested = packet.buttons & BUTTON_ARM;
  const bool disarmRequested = packet.buttons & BUTTON_DISARM;
  if (disarmRequested) {
    return false;
  }
  if (!armRequested) {
    return isArmed;
  }
  // Require throttle low to arm
  return packet.throttle < (MOTOR_MIN_US + 50);
}

uint16_t constrainMotor(float value) {
  return static_cast<uint16_t>(constrain(value, MOTOR_MIN_US, MOTOR_MAX_US));
}

float mapControl(int16_t value, int16_t inMin, int16_t inMax, float outMin, float outMax) {
  float ratio = (float)(value - inMin) / (float)(inMax - inMin);
  ratio = constrain(ratio, 0.0f, 1.0f);
  return outMin + ratio * (outMax - outMin);
}

void updateAltitudeHold() {
  static bool latched = false;
  bool switchOn = controlPacket.switches & SWITCH_ALT_HOLD;
  if (switchOn && !latched) {
    altitudeTargetM = ms5611::altitudeMeters();
  }
  altitudeHold = switchOn;
  latched = switchOn;
}

float throttlePid(float target, float current, float dt) {
  static PID pidAlt{1.4f, 0.6f, 0.0f, -200.0f, 200.0f};
  if (!altitudeHold) {
    pidAlt.reset();
    return 0.0f;
  }
  return pidAlt.compute(target, current, dt);
}

void loop() {
  static uint32_t lastLoopTime = micros();
  uint32_t now = micros();
  uint32_t elapsed = now - lastLoopTime;
  if (elapsed < LOOP_PERIOD_US) {
    return;
  }
  lastLoopTime = now;
  float dt = elapsed / 1e6f;

  // Update sensors
  updateAttitude(dt);
  ms5611::update();

  // Radio
  while (radio.available()) {
    radio.read(&controlPacket, sizeof(controlPacket));
    if (rc::validate(controlPacket)) {
      radioHasSignal = true;
      lastSignalMs = millis();
    }
  }

  if (radioHasSignal && millis() - lastSignalMs > SIGNAL_TIMEOUT_MS) {
    radioHasSignal = false;
  }

  if (radioHasSignal) {
    bool should_be_armed = shouldArm(controlPacket);
    if (should_be_armed != isArmed) {
      isArmed = should_be_armed;
      signalArming(isArmed);
      if (!isArmed) {
        pidRoll.reset();
        pidPitch.reset();
        pidYaw.reset();
        setAllMotors(MOTOR_MIN_US);
      }
    }
  } else if (isArmed) {
    isArmed = false;
    signalArming(false);
    setAllMotors(MOTOR_MIN_US);
  }

  if (!isArmed) {
    if (controlPacket.buttons & BUTTON_BEEP) {
      tone(PIN_BUZZER, 2600, 120);
    }
    return;
  }

  float throttle = constrain(controlPacket.throttle, MOTOR_MIN_US, MOTOR_MAX_US);

  // Convert control inputs to desired angles/rates
  float targetRoll = mapControl(controlPacket.roll, -500, 500, -25.0f, 25.0f);
  float targetPitch = mapControl(controlPacket.pitch, -500, 500, -25.0f, 25.0f);
  float targetYawRate = mapControl(controlPacket.yaw, -500, 500, -150.0f, 150.0f);

  float rollOutput = pidRoll.compute(targetRoll, attitude.roll_deg, dt);
  float pitchOutput = pidPitch.compute(targetPitch, attitude.pitch_deg, dt);
  float yawOutput = pidYaw.compute(targetYawRate, attitude.yaw_rate_dps, dt);

  updateAltitudeHold();
  float altCorrection = throttlePid(altitudeTargetM, ms5611::altitudeMeters(), dt);
  throttle += altCorrection;
  throttle = constrain(throttle, (float)MOTOR_MIN_US, (float)MOTOR_MAX_US);

  float motorMixFL = throttle + pitchOutput - rollOutput - yawOutput;
  float motorMixFR = throttle + pitchOutput + rollOutput + yawOutput;
  float motorMixRR = throttle - pitchOutput + rollOutput - yawOutput;
  float motorMixRL = throttle - pitchOutput - rollOutput + yawOutput;

  uint16_t outFL = constrainMotor(motorMixFL);
  uint16_t outFR = constrainMotor(motorMixFR);
  uint16_t outRR = constrainMotor(motorMixRR);
  uint16_t outRL = constrainMotor(motorMixRL);

  motorFL.writeMicroseconds(outFL);
  motorFR.writeMicroseconds(outFR);
  motorRR.writeMicroseconds(outRR);
  motorRL.writeMicroseconds(outRL);

  if (controlPacket.buttons & BUTTON_BEEP) {
    tone(PIN_BUZZER, 2600, 50);
  } else {
    noTone(PIN_BUZZER);
  }

  // Prepare telemetry
  rc::TelemetryPacket telemetry;
  telemetry.roll_deg = attitude.roll_deg * 10;
  telemetry.pitch_deg = attitude.pitch_deg * 10;
  telemetry.yaw_rate_dps = attitude.yaw_rate_dps * 10;
  telemetry.altitude_cm = ms5611::altitudeMeters() * 100;
  telemetry.status_flags = (isArmed ? 0x0001 : 0x0000) |
                           (altitudeHold ? 0x0002 : 0x0000) |
                           (radioHasSignal ? 0x0004 : 0x0000);
  telemetry.battery_mv = controlPacket.aux1; // placeholder for voltage sensor
  rc::finalize(telemetry);
  radio.writeAckPayload(kRadioPipeController, &telemetry, sizeof(telemetry));
}
