/**
 * DIY Quadcopter Flight Controller
 * --------------------------------
 * Target board : Arduino Nano (ATmega328P @16MHz)
 * Radio        : nRF24L01+  (CE=D4, CSN=D10)
 * IMU          : MPU6050    (INT=D2, I2C=A4/A5)
 * Barometer    : GY-63 MS5611 (I2C=A4/A5)
 * Motors       : FL=D3, FR=D5, RR=D6, RL=D7 (via ESCs)
 * Buzzer       : D8  (active low)
 * Status LED   : D13 (built-in LED)  <-- D7 reserved for rear-left motor PWM
 *
 * Libraries required (Arduino Library Manager):
 * - RF24 by TMRh20
 * - Servo (built-in)
 * - I2Cdevlib - MPU6050 (by Electronic Cats / Jeff Rowberg)
 * - I2Cdevlib - I2Cdev (dependency for the above)
 * - MS5611 by Rob Tillaart (or compatible "MS5611.h" interface)
 *
 * This sketch implements a PID flight controller with complementary-filter
 * attitude estimation using the MPU6050 library and barometric altitude trending
 * via the MS5611 library. It expects an RC transmitter that sends the RcPacket
 * defined below over the nRF24L01+.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <type_traits>
#include <I2Cdev.h>
#include <MPU6050.h>
#include "MS5611.h"
#include <RF24.h>
#include <Servo.h>

#include "../shared/RcPacket.h"

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
const uint8_t RADIO_CHANNEL = 115; // ensure RC transmitter matches this channel
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
const byte RADIO_ADDRESS_RX[6] = "DRN1";
const byte RADIO_ADDRESS_TX[6] = "RC01";

// I2C addresses (AD0 on MPU6050 low -> 0x68, CSB on MS5611 high -> 0x77)
const uint8_t MPU6050_I2C_ADDRESS = 0x68;
#if defined(MS5611_ADDRESS_HIGH)
const uint8_t MS5611_I2C_ADDRESS = MS5611_ADDRESS_HIGH;
#else
const uint8_t MS5611_I2C_ADDRESS = 0x77;
#endif

MPU6050 imu(MPU6050_I2C_ADDRESS);
MS5611 ms5611;

// ---------------------------------------------------------------------------
// Helper templates to gracefully support multiple MS5611 library variants
// ---------------------------------------------------------------------------
inline bool interpretMs5611Result(bool result) { return result; }
inline bool interpretMs5611Result(int result) { return (result == 0) || (result > 0); }
inline bool interpretMs5611Result(uint8_t result) { return (result == 0) || (result == 1); }
inline bool interpretMs5611Result(float) { return true; }
template <typename T>
inline bool interpretMs5611Result(T) { return true; }

template <typename Sensor>
auto ms5611SetAddressIfAvailable(Sensor &sensor, uint8_t address, int)
    -> decltype(sensor.setAddress(address), void()) {
  sensor.setAddress(address);
}
template <typename Sensor>
void ms5611SetAddressIfAvailable(Sensor &, uint8_t, ...) {}

template <typename Sensor, typename Oversample>
auto ms5611SetOversamplingIfAvailable(Sensor &sensor, Oversample value, int)
    -> decltype(sensor.setOversampling(value), void()) {
  sensor.setOversampling(value);
}
template <typename Sensor, typename Oversample>
void ms5611SetOversamplingIfAvailable(Sensor &, Oversample, ...) {}

template <typename Sensor>
bool ms5611BeginDispatchAddr(Sensor &sensor, uint8_t address, std::false_type) {
  return interpretMs5611Result(sensor.begin(address));
}
template <typename Sensor>
bool ms5611BeginDispatchAddr(Sensor &sensor, uint8_t address, std::true_type) {
  (void)address;
  sensor.begin(address);
  return true;
}
template <typename Sensor>
auto ms5611Begin(Sensor &sensor, uint8_t address, int)
    -> decltype(sensor.begin(address), bool()) {
  typedef typename std::is_void<decltype(sensor.begin(address))>::type IsVoid;
  return ms5611BeginDispatchAddr(sensor, address, IsVoid{});
}

template <typename Sensor>
bool ms5611BeginDispatchNoAddr(Sensor &sensor, std::false_type) {
  return interpretMs5611Result(sensor.begin());
}
template <typename Sensor>
bool ms5611BeginDispatchNoAddr(Sensor &sensor, std::true_type) {
  sensor.begin();
  return true;
}
template <typename Sensor>
bool ms5611Begin(Sensor &sensor, uint8_t, ...) {
  typedef typename std::is_void<decltype(sensor.begin())>::type IsVoid;
  return ms5611BeginDispatchNoAddr(sensor, IsVoid{});
}

template <typename Sensor>
bool ms5611ReadDispatch(Sensor &sensor, std::false_type) {
  return interpretMs5611Result(sensor.read());
}
template <typename Sensor>
bool ms5611ReadDispatch(Sensor &sensor, std::true_type) {
  sensor.read();
  return true;
}
template <typename Sensor>
bool ms5611Read(Sensor &sensor) {
  typedef typename std::is_void<decltype(sensor.read())>::type IsVoid;
  return ms5611ReadDispatch(sensor, IsVoid{});
}

// ---------------------------------------------------------------------------
// Sensor constants and state
// ---------------------------------------------------------------------------
constexpr float ACCEL_SCALE = 16384.0f; // LSB/g for +/-2g
constexpr float GYRO_SCALE = 131.0f;    // LSB/(deg/s) for +/-250 deg/s

struct ImuRawSample {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

ImuRawSample imuRaw{};

float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float accelBiasX = 0.0f, accelBiasY = 0.0f, accelBiasZ = 0.0f;

// Attitude state
float rollAngle = 0.0f;  // in degrees
float pitchAngle = 0.0f; // in degrees
float yawRate = 0.0f;    // deg/s (integrated yaw not used for control)

// Altitude estimate
float altitudeMeters = 0.0f;
float basePressurePa = 101325.0f;
float lastTemperatureC = 20.0f;

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
uint32_t lastBaroSampleMillis = 0;

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------
template <typename T> T clamp(T value, T minVal, T maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

float toPascal(float pressureReading) {
  // Most MS5611 libraries return hPa (mbar). If already Pa, leave unchanged.
  if (pressureReading < 2000.0f) {
    return pressureReading * 100.0f;
  }
  return pressureReading;
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
// Sensor helpers
// ---------------------------------------------------------------------------
bool initImu() {
  imu.initialize();
  imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  imu.setDLPFMode(MPU6050_DLPF_BW_42);
  return imu.testConnection();
}

void readImuSample(ImuRawSample &sample) {
  imu.getMotion6(&sample.ax, &sample.ay, &sample.az, &sample.gx, &sample.gy, &sample.gz);
}

bool initBarometer() {
  ms5611SetAddressIfAvailable(ms5611, MS5611_I2C_ADDRESS, 0);
  bool ok = ms5611Begin(ms5611, MS5611_I2C_ADDRESS, 0);
#if defined(MS5611_OSR_ULTRA_HIGH)
  ms5611SetOversamplingIfAvailable(ms5611, MS5611_OSR_ULTRA_HIGH, 0);
#elif defined(MS5611_ULTRA_HIGH)
  ms5611SetOversamplingIfAvailable(ms5611, MS5611_ULTRA_HIGH, 0);
#endif
  float pressureSum = 0.0f;
  float tempSum = 0.0f;
  const int samples = 20;
  for (int i = 0; i < samples; ++i) {
    if (ms5611Read(ms5611)) {
      pressureSum += toPascal(ms5611.getPressure());
      tempSum += ms5611.getTemperature();
    }
    delay(10);
  }
  if (pressureSum > 0.0f) {
    basePressurePa = pressureSum / samples;
  }
  if (tempSum > 0.0f) {
    lastTemperatureC = tempSum / samples;
  }
  return ok;
}

// ---------------------------------------------------------------------------
// System initialisation
// ---------------------------------------------------------------------------
void calibrateIMU() {
  const int samples = 500;
  float gx = 0.0f, gy = 0.0f, gz = 0.0f;
  float ax = 0.0f, ay = 0.0f, az = 0.0f;
  for (int i = 0; i < samples; ++i) {
    readImuSample(imuRaw);
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
  accelBiasZ = az / samples - ACCEL_SCALE; // assume +1g on Z
}

void initRadio() {
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(RADIO_CHANNEL);
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
  readImuSample(imuRaw);

  float ax = (imuRaw.ax - accelBiasX) / ACCEL_SCALE;
  float ay = (imuRaw.ay - accelBiasY) / ACCEL_SCALE;
  float az = (imuRaw.az - accelBiasZ) / ACCEL_SCALE;
  float gx = (imuRaw.gx - gyroBiasX) / GYRO_SCALE;
  float gy = (imuRaw.gy - gyroBiasY) / GYRO_SCALE;
  float gz = (imuRaw.gz - gyroBiasZ) / GYRO_SCALE;

  float accelRoll = atan2f(ay, az) * 57.2958f;
  float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;

  rollAngle = 0.98f * (rollAngle + gx * dt) + 0.02f * accelRoll;
  pitchAngle = 0.98f * (pitchAngle + gy * dt) + 0.02f * accelPitch;
  yawRate = gz;
}

void updateBarometer() {
  const uint32_t now = millis();
  if (now - lastBaroSampleMillis < 25) {
    return;
  }
  lastBaroSampleMillis = now;

  if (ms5611Read(ms5611)) {
    float pressurePa = toPascal(ms5611.getPressure());
    lastTemperatureC = ms5611.getTemperature();
    if (pressurePa > 10000.0f) {
      float altitude = 44330.0f * (1.0f - powf(pressurePa / basePressurePa, 0.1903f));
      altitudeMeters = 0.95f * altitudeMeters + 0.05f * altitude;
    }
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
  Wire.setClock(400000); // fast-mode I2C for sensors
  pinMode(PIN_IMU_INT, INPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  setBuzzer(false);
  digitalWrite(PIN_STATUS_LED, LOW);

  bool imuOk = initImu();
  calibrateIMU();
  bool baroOk = initBarometer();

  initRadio();

  motorFL.attach(PIN_MOTOR_FL);
  motorFR.attach(PIN_MOTOR_FR);
  motorRR.attach(PIN_MOTOR_RR);
  motorRL.attach(PIN_MOTOR_RL);
  setAllMotorsIdle();

  lastPacketMicros = micros();
  failsafeActive = true;

  if (!imuOk || !baroOk) {
    Serial.println(F("[WARN] Sensor init failed. Check wiring and addresses."));
  }
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
