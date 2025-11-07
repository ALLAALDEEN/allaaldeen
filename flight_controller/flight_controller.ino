/**
 * DIY Quadcopter Flight Controller
 * --------------------------------
 * Target board : Arduino Nano (ATmega328P @16MHz)
 * Radio        : nRF24L01+  (CE=D4, CSN=D10)
 * IMU          : MPU6050    (INT=D2)
 * Barometer    : GY-63 MS5611 (I2C)
 * Motors       : FL=D3, FR=D5, RR=D6, RL=D7 (via ESCs)
 * Buzzer       : D8  (active low)
 * Status LED   : D13 (built-in LED)  <-- D7 reserved for rear-left motor PWM
 *
 * Libraries required:
 * - RF24 by TMRh20
 * - Servo (built-in)
 *
 * This sketch implements a basic PID flight controller with complementary-filter
 * attitude estimation and a simple altitude trend estimate. It expects a matching
 * RC transmitter sketch that sends the RcPacket structure defined below.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// ---------------------------------------------------------------------------
// Hardware pin definitions
// ---------------------------------------------------------------------------
const uint8_t PIN_RF_CE = 4;
const uint8_t PIN_RF_CSN = 10;
const uint8_t PIN_IMU_INT = 2;

const uint8_t PIN_MOTOR_FL = 3;
const uint8_t PIN_MOTOR_FR = 5;
const uint8_t PIN_MOTOR_RR = 6;
const uint8_t PIN_MOTOR_RL = 7;

const uint8_t PIN_BUZZER = 8;
const uint8_t PIN_STATUS_LED = LED_BUILTIN; // D13 on Nano

// ---------------------------------------------------------------------------
// Radio configuration
// ---------------------------------------------------------------------------
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
const byte RADIO_ADDRESS_RX[6] = "DRN1";
const byte RADIO_ADDRESS_TX[6] = "RC01";

struct __attribute__((packed)) RcPacket {
  uint32_t sequence;
  uint16_t throttle;   // 0..1000
  int16_t roll;        // -500..500 (scaled degrees reference)
  int16_t pitch;       // -500..500
  int16_t yaw;         // -500..500 (yaw rate command)
  int16_t aux1;        // -500..500 (pot 1 centered)
  int16_t aux2;        // -500..500 (pot 2 centered)
  uint8_t buttons;     // bits 0..3 = buttons 1..4, bit set = pressed
  uint8_t switches;    // bit0 = SW1 (arming), bit1 = SW2 (mode)
  uint16_t checksum;
};

volatile bool imuDataReady = false;

// ---------------------------------------------------------------------------
// Sensor helpers (MPU6050 + MS5611)
// ---------------------------------------------------------------------------
namespace imu {
constexpr uint8_t ADDRESS = 0x68;
constexpr float ACCEL_SCALE = 16384.0f; // LSB/g for +/-2g
constexpr float GYRO_SCALE = 131.0f;    // LSB/(deg/s) for +/-250 deg/s

struct Sample {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};
} // namespace imu

namespace baro {
constexpr uint8_t ADDRESS = 0x77;
constexpr uint8_t CMD_RESET = 0x1E;
constexpr uint8_t CMD_PROM_BASE = 0xA2; // C1..C6
constexpr uint8_t CMD_CONVERT_D1 = 0x48; // OSR=4096
constexpr uint8_t CMD_CONVERT_D2 = 0x58; // OSR=4096
constexpr uint8_t CMD_READ_ADC = 0x00;

struct Calibration {
  uint16_t c1_sens;
  uint16_t c2_off;
  uint16_t c3_tcs;
  uint16_t c4_tco;
  uint16_t c5_tref;
  uint16_t c6_tempsens;
};
} // namespace baro

imu::Sample imuRaw{};
baro::Calibration baroCal{};

float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float accelBiasX = 0.0f, accelBiasY = 0.0f, accelBiasZ = 0.0f;

// Attitude state
float rollAngle = 0.0f;  // in degrees
float pitchAngle = 0.0f; // in degrees
float yawRate = 0.0f;    // deg/s (integrated yaw not used for control)

// Altitude estimate
float altitudeMeters = 0.0f;
float basePressure = 101325.0f; // Pa

// ---------------------------------------------------------------------------
// Motor control
// ---------------------------------------------------------------------------
Servo motorFL;
Servo motorFR;
Servo motorRR;
Servo motorRL;

constexpr float SERVO_MIN_US = 1000.0f;
constexpr float SERVO_MAX_US = 2000.0f;
constexpr float SERVO_ARM_US = 1000.0f;

bool isArmed = false;
bool failsafeActive = true;

// PID state
struct PIDState {
  float kp;
  float ki;
  float kd;
  float integral;
  float previousError;
};

PIDState rollPid{1.8f, 0.015f, 35.0f, 0.0f, 0.0f};
PIDState pitchPid{1.8f, 0.015f, 35.0f, 0.0f, 0.0f};
PIDState yawPid{2.0f, 0.0f, 10.0f, 0.0f, 0.0f};

// Incoming RC data
RcPacket rcPacket{};
uint32_t lastPacketMicros = 0;
uint32_t lastBlinkMillis = 0;
uint32_t lastBaroConversionMillis = 0;
bool baroRequestingPressure = true;
uint32_t baroLastRaw = 0;

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------
uint16_t computeChecksum(const RcPacket &packet) {
  const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&packet);
  uint16_t sum = 0;
  for (size_t i = 0; i < sizeof(RcPacket) - sizeof(packet.checksum); ++i) {
    sum += ptr[i];
  }
  return sum;
}

template <typename T> T clamp(T value, T minVal, T maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

void writeMotorsMicroseconds(float fl, float fr, float rr, float rl) {
  motorFL.writeMicroseconds(static_cast<int>(clamp(fl, SERVO_MIN_US, SERVO_MAX_US)));
  motorFR.writeMicroseconds(static_cast<int>(clamp(fr, SERVO_MIN_US, SERVO_MAX_US)));
  motorRR.writeMicroseconds(static_cast<int>(clamp(rr, SERVO_MIN_US, SERVO_MAX_US)));
  motorRL.writeMicroseconds(static_cast<int>(clamp(rl, SERVO_MIN_US, SERVO_MAX_US)));
}

void setAllMotorsIdle() {
  writeMotorsMicroseconds(SERVO_ARM_US, SERVO_ARM_US, SERVO_ARM_US, SERVO_ARM_US);
}

void setBuzzer(bool on) {
  digitalWrite(PIN_BUZZER, on ? LOW : HIGH); // active low
}

// ---------------------------------------------------------------------------
// Sensor low-level functions
// ---------------------------------------------------------------------------
void imuWriteByte(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imu::ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

void imuReadBytes(uint8_t reg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(imu::ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(imu::ADDRESS, count, true);
  for (uint8_t i = 0; i < count && Wire.available(); ++i) {
    dest[i] = Wire.read();
  }
}

bool imuInit() {
  imuWriteByte(0x6B, 0x00); // Wake up, use internal oscillator
  delay(100);
  imuWriteByte(0x1B, 0x00); // Gyro +/-250 deg/s
  imuWriteByte(0x1C, 0x00); // Accel +/-2 g
  imuWriteByte(0x1A, 0x03); // DLPF ~43 Hz
  return true;
}

void imuReadSample(imu::Sample &sample) {
  uint8_t buffer[14];
  imuReadBytes(0x3B, 14, buffer);
  sample.ax = (buffer[0] << 8) | buffer[1];
  sample.ay = (buffer[2] << 8) | buffer[3];
  sample.az = (buffer[4] << 8) | buffer[5];
  sample.gx = (buffer[8] << 8) | buffer[9];
  sample.gy = (buffer[10] << 8) | buffer[11];
  sample.gz = (buffer[12] << 8) | buffer[13];
}

void baroWrite(uint8_t cmd) {
  Wire.beginTransmission(baro::ADDRESS);
  Wire.write(cmd);
  Wire.endTransmission();
}

uint32_t baroReadADC() {
  Wire.beginTransmission(baro::ADDRESS);
  Wire.write(baro::CMD_READ_ADC);
  Wire.endTransmission();
  Wire.requestFrom(baro::ADDRESS, (uint8_t)3);
  uint32_t value = 0;
  if (Wire.available() == 3) {
    value = (uint32_t)Wire.read() << 16;
    value |= (uint32_t)Wire.read() << 8;
    value |= Wire.read();
  }
  return value;
}

bool baroInit() {
  baroWrite(baro::CMD_RESET);
  delay(4);
  Wire.beginTransmission(baro::ADDRESS);
  for (uint8_t i = 0; i < 6; ++i) {
    Wire.write(baro::CMD_PROM_BASE + i * 2);
    Wire.endTransmission(false);
    Wire.requestFrom(baro::ADDRESS, (uint8_t)2);
    if (Wire.available() == 2) {
      uint16_t value = (Wire.read() << 8) | Wire.read();
      switch (i) {
        case 0: baroCal.c1_sens = value; break;
        case 1: baroCal.c2_off = value; break;
        case 2: baroCal.c3_tcs = value; break;
        case 3: baroCal.c4_tco = value; break;
        case 4: baroCal.c5_tref = value; break;
        case 5: baroCal.c6_tempsens = value; break;
      }
    }
  }
  baroWrite(baro::CMD_CONVERT_D1);
  lastBaroConversionMillis = millis();
  baroRequestingPressure = true;
  return true;
}

float baroCalculateAltitude(uint32_t d1, uint32_t d2) {
  // Calibration algorithm from datasheet
  int32_t dT = (int32_t)d2 - ((int32_t)baroCal.c5_tref << 8);
  int64_t OFF = ((int64_t)baroCal.c2_off << 16) + ((int64_t)baroCal.c4_tco * dT) / 128;
  int64_t SENS = ((int64_t)baroCal.c1_sens << 15) + ((int64_t)baroCal.c3_tcs * dT) / 256;
  int32_t TEMP = 2000 + ((int64_t)dT * baroCal.c6_tempsens) / 8388608;

  int32_t T2 = 0;
  int64_t OFF2 = 0;
  int64_t SENS2 = 0;
  if (TEMP < 2000) {
    T2 = (dT * dT) >> 31;
    OFF2 = 5LL * ((TEMP - 2000) * (TEMP - 2000)) >> 1;
    SENS2 = 5LL * ((TEMP - 2000) * (TEMP - 2000)) >> 2;
    if (TEMP < -1500) {
      OFF2 += 7LL * ((TEMP + 1500) * (TEMP + 1500));
      SENS2 += 11LL * ((TEMP + 1500) * (TEMP + 1500)) >> 1;
    }
  }
  TEMP -= T2;
  OFF -= OFF2;
  SENS -= SENS2;

  int32_t P = (((int64_t)d1 * SENS) / 2097152 - OFF) / 32768;
  float pressure = P; // Pa
  // Barometric formula (approx)
  float altitude = 44330.0f * (1.0f - pow(pressure / basePressure, 0.1903f));
  return altitude;
}

// ---------------------------------------------------------------------------
// System initialisation
// ---------------------------------------------------------------------------
void calibrateIMU() {
  const int samples = 500;
  float gx = 0.0f, gy = 0.0f, gz = 0.0f;
  float ax = 0.0f, ay = 0.0f, az = 0.0f;
  for (int i = 0; i < samples; ++i) {
    imuReadSample(imuRaw);
    gx += imuRaw.gx;
    gy += imuRaw.gy;
    gz += imuRaw.gz;
    ax += imuRaw.ax;
    ay += imuRaw.ay;
    az += imuRaw.az;
    delay(5);
  }
  gyroBiasX = gx / samples;
  gyroBiasY = gy / samples;
  gyroBiasZ = gz / samples;
  accelBiasX = ax / samples;
  accelBiasY = ay / samples;
  accelBiasZ = az / samples - imu::ACCEL_SCALE; // assume +1g on Z
}

void initRadio() {
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(115);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.openWritingPipe(RADIO_ADDRESS_TX);
  radio.openReadingPipe(1, RADIO_ADDRESS_RX);
  radio.startListening();
}

// ---------------------------------------------------------------------------
// Control helpers
// ---------------------------------------------------------------------------
float runPid(PIDState &pid, float error, float dt, float iLimit, float outputLimit) {
  pid.integral += error * dt;
  pid.integral = clamp(pid.integral, -iLimit, iLimit);
  float derivative = (error - pid.previousError) / dt;
  pid.previousError = error;
  float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
  return clamp(output, -outputLimit, outputLimit);
}

void updateAttitude(float dt) {
  imuReadSample(imuRaw);

  float ax = (imuRaw.ax - accelBiasX) / imu::ACCEL_SCALE;
  float ay = (imuRaw.ay - accelBiasY) / imu::ACCEL_SCALE;
  float az = (imuRaw.az - accelBiasZ) / imu::ACCEL_SCALE;
  float gx = (imuRaw.gx - gyroBiasX) / imu::GYRO_SCALE;
  float gy = (imuRaw.gy - gyroBiasY) / imu::GYRO_SCALE;
  float gz = (imuRaw.gz - gyroBiasZ) / imu::GYRO_SCALE;

  float accelRoll = atan2f(ay, az) * 57.2958f;
  float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;

  rollAngle = 0.98f * (rollAngle + gx * dt) + 0.02f * accelRoll;
  pitchAngle = 0.98f * (pitchAngle + gy * dt) + 0.02f * accelPitch;
  yawRate = gz;
}

void updateBarometer() {
  const uint32_t now = millis();
  if (now - lastBaroConversionMillis < 10) {
    return;
  }
  lastBaroConversionMillis = now;

  if (baroRequestingPressure) {
    baroLastRaw = baroReadADC(); // D1 pressure
    baroWrite(baro::CMD_CONVERT_D2);
    baroRequestingPressure = false;
  } else {
    uint32_t d2 = baroReadADC(); // temperature
    float altitude = baroCalculateAltitude(baroLastRaw, d2);
    altitudeMeters = 0.95f * altitudeMeters + 0.05f * altitude;
    baroWrite(baro::CMD_CONVERT_D1);
    baroRequestingPressure = true;
  }
}

void handleRadio() {
  while (radio.available()) {
    RcPacket incoming{};
    radio.read(&incoming, sizeof(incoming));
    if (incoming.checksum == computeChecksum(incoming)) {
      rcPacket = incoming;
      lastPacketMicros = micros();
      failsafeActive = false;
    }
  }
}

void updateFailsafeState() {
  const uint32_t nowUs = micros();
  if ((nowUs - lastPacketMicros) > 200000) { // >200 ms
    failsafeActive = true;
    isArmed = false;
  }
}

void updateArmingLogic() {
  const bool armSwitch = rcPacket.switches & 0x01;
  if (!armSwitch) {
    isArmed = false;
    return;
  }
  if (failsafeActive) {
    isArmed = false;
    return;
  }
  if (!isArmed) {
    if (rcPacket.throttle < 50 && abs(rcPacket.roll) < 30 && abs(rcPacket.pitch) < 30) {
      isArmed = true;
      rollPid.integral = pitchPid.integral = yawPid.integral = 0.0f;
      rollPid.previousError = pitchPid.previousError = yawPid.previousError = 0.0f;
    }
  }
}

void runControl(float dt) {
  float throttleNorm = rcPacket.throttle / 1000.0f;
  throttleNorm = clamp(throttleNorm, 0.0f, 1.0f);

  const float MAX_ANGLE = 40.0f; // degrees
  const float MAX_YAW_RATE = 150.0f; // deg/s

  float targetRoll = (rcPacket.roll / 500.0f) * MAX_ANGLE;
  float targetPitch = (rcPacket.pitch / 500.0f) * MAX_ANGLE;
  float targetYawRate = (rcPacket.yaw / 500.0f) * MAX_YAW_RATE;

  float rollError = targetRoll - rollAngle;
  float pitchError = targetPitch - pitchAngle;
  float yawError = targetYawRate - yawRate;

  float rollAdjust = runPid(rollPid, rollError, dt, 200.0f, 400.0f);
  float pitchAdjust = runPid(pitchPid, pitchError, dt, 200.0f, 400.0f);
  float yawAdjust = runPid(yawPid, yawError, dt, 100.0f, 200.0f);

  // Height hold stub: reserved for future use via rcPacket.aux1/aux2
  float baseThrottleUs = SERVO_MIN_US + throttleNorm * (SERVO_MAX_US - SERVO_MIN_US);

  float motorFLus = baseThrottleUs + pitchAdjust + rollAdjust - yawAdjust;
  float motorFRus = baseThrottleUs + pitchAdjust - rollAdjust + yawAdjust;
  float motorRRus = baseThrottleUs - pitchAdjust - rollAdjust - yawAdjust;
  float motorRLus = baseThrottleUs - pitchAdjust + rollAdjust + yawAdjust;

  if (!isArmed || failsafeActive) {
    setAllMotorsIdle();
  } else {
    writeMotorsMicroseconds(motorFLus, motorFRus, motorRRus, motorRLus);
  }
}

void updateIndicators() {
  const uint32_t now = millis();
  if (failsafeActive) {
    if ((now / 200) % 2 == 0) {
      digitalWrite(PIN_STATUS_LED, HIGH);
      setBuzzer(true);
    } else {
      digitalWrite(PIN_STATUS_LED, LOW);
      setBuzzer(false);
    }
    return;
  }

  if (!isArmed) {
    if (now - lastBlinkMillis > 500) {
      lastBlinkMillis = now;
      digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
    }
    setBuzzer(false);
  } else {
    digitalWrite(PIN_STATUS_LED, HIGH);
    setBuzzer(false);
  }
}

// ---------------------------------------------------------------------------
// Arduino lifecycle
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(PIN_IMU_INT, INPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  setBuzzer(false);
  digitalWrite(PIN_STATUS_LED, LOW);

  imuInit();
  calibrateIMU();
  baroInit();

  initRadio();

  motorFL.attach(PIN_MOTOR_FL);
  motorFR.attach(PIN_MOTOR_FR);
  motorRR.attach(PIN_MOTOR_RR);
  motorRL.attach(PIN_MOTOR_RL);
  setAllMotorsIdle();

  lastPacketMicros = micros();
  failsafeActive = true;
}

void loop() {
  static uint32_t previousMicros = micros();
  uint32_t currentMicros = micros();
  float dt = (currentMicros - previousMicros) / 1e6f;
  if (dt <= 0.0f || dt > 0.05f) {
    dt = 0.01f;
  }
  previousMicros = currentMicros;

  handleRadio();
  updateFailsafeState();
  updateArmingLogic();
  updateAttitude(dt);
  updateBarometer();
  runControl(dt);
  updateIndicators();
}
