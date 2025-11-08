#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <math.h>

#include "../shared/RadioPacket.h"

using namespace DroneLink;

// -----------------------------------------------------------------------------
// Hardware mapping (Flight Controller side)
// -----------------------------------------------------------------------------
constexpr uint8_t RADIO_CE_PIN = 4;
constexpr uint8_t RADIO_CSN_PIN = 10;

constexpr uint8_t MPU_INT_PIN = 2;

constexpr uint8_t MOTOR_FL_PIN = 3;
constexpr uint8_t MOTOR_FR_PIN = 5;
constexpr uint8_t MOTOR_RR_PIN = 6;
constexpr uint8_t MOTOR_RL_PIN = 7;

constexpr uint8_t BUZZER_PIN = 8;
constexpr uint8_t STATUS_LED_PIN = 13;  // Change to 7 if you have a dedicated LED pin.

// ESC pulse limits (microseconds)
constexpr int MOTOR_MIN_US = 1000;
constexpr int MOTOR_IDLE_US = 1035;
constexpr int MOTOR_MAX_US = 2000;
constexpr float THROTTLE_SCALE = 0.85f;  // Reserve headroom for PID corrections
constexpr float PID_HEADROOM_US = 200.0f;

// Flight behavior
constexpr float MAX_ANGLE_DEG = 35.0f;
constexpr float MAX_YAW_RATE_DPS = 160.0f;
constexpr float MAX_ALTITUDE_METERS = 3.0f;

constexpr unsigned long RADIO_TIMEOUT_MS = 200;
constexpr unsigned long FAILSAFE_MOTOR_SPINDOWN_MS = 500;
constexpr unsigned long LOOP_INTERVAL_US = 4000;  // 250 Hz control loop

// -----------------------------------------------------------------------------
// Simple MS5611 driver (blocking, OSR=4096)
// -----------------------------------------------------------------------------
class MS5611Sensor {
 public:
  bool begin(uint8_t address = 0x77) {
    address_ = address;
    reset();
    delay(3);
    return readProm();
  }

  bool read(float &temperatureC, float &pressurePa) {
    uint32_t D1 = 0;
    uint32_t D2 = 0;
    if (!startConversion(0x48) || !readADC(D1)) {
      return false;
    }
    if (!startConversion(0x58) || !readADC(D2)) {
      return false;
    }

    int64_t dT = static_cast<int64_t>(D2) - (static_cast<int64_t>(prom_[5]) << 8);
    int64_t TEMP = 2000 + ((dT * prom_[6]) >> 23);

    int64_t OFF = (static_cast<int64_t>(prom_[2]) << 16) + ((static_cast<int64_t>(prom_[4]) * dT) >> 7);
    int64_t SENS = (static_cast<int64_t>(prom_[1]) << 15) + ((static_cast<int64_t>(prom_[3]) * dT) >> 8);

    int32_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;
    if (TEMP < 2000) {
      int64_t temp_minus = TEMP - 2000;
      int64_t temp_sq = temp_minus * temp_minus;
      T2 = static_cast<int32_t>((dT * dT) >> 31);
      OFF2 = (5 * temp_sq) >> 1;
      SENS2 = (5 * temp_sq) >> 2;
      if (TEMP < -1500) {
        int64_t temp_minus2 = TEMP + 1500;
        temp_minus2 = temp_minus2 * temp_minus2;
        OFF2 += 7 * temp_minus2;
        SENS2 += (11 * temp_minus2) >> 1;
      }
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    int64_t P = (((static_cast<int64_t>(D1) * SENS) >> 21) - OFF) >> 15;

    temperatureC = static_cast<float>(TEMP) / 100.0f;
    pressurePa = static_cast<float>(P);  // Output is already in Pascals.
    return true;
  }

 private:
  uint8_t address_ = 0x77;
  uint16_t prom_[7]{};

  void reset() {
    Wire.beginTransmission(address_);
    Wire.write(0x1E);
    Wire.endTransmission();
  }

  bool readProm() {
    for (uint8_t i = 0; i < 6; ++i) {
      prom_[i + 1] = readPromWord(0xA2 + (i * 2));
      if (prom_[i + 1] == 0) {
        return false;
      }
    }
    return true;
  }

  uint16_t readPromWord(uint8_t cmd) {
    Wire.beginTransmission(address_);
    Wire.write(cmd);
    if (Wire.endTransmission() != 0) {
      return 0;
    }
    Wire.requestFrom(address_, static_cast<uint8_t>(2));
    if (Wire.available() < 2) {
      return 0;
    }
    uint16_t value = static_cast<uint16_t>(Wire.read()) << 8;
    value |= Wire.read();
    return value;
  }

  bool startConversion(uint8_t command) {
    Wire.beginTransmission(address_);
    Wire.write(command);
    if (Wire.endTransmission() != 0) {
      return false;
    }
    delay(10);  // 9.04 ms for OSR 4096
    return true;
  }

  bool readADC(uint32_t &value) {
    Wire.beginTransmission(address_);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) {
      return false;
    }
    Wire.requestFrom(address_, static_cast<uint8_t>(3));
    if (Wire.available() < 3) {
      return false;
    }
    value = static_cast<uint32_t>(Wire.read()) << 16;
    value |= static_cast<uint32_t>(Wire.read()) << 8;
    value |= Wire.read();
    return true;
  }
};

// -----------------------------------------------------------------------------
// IMU and barometer globals
// -----------------------------------------------------------------------------
#include <MPU6050.h>

MPU6050 imu;
MS5611Sensor barometer;

float gyroOffset[3] = {0, 0, 0};
float accelOffset[3] = {0, 0, 0};

struct Orientation {
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
};

struct Rates {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

Orientation attitude{};
Rates angularRate{};

bool imuReady = false;
bool baroPresent = false;
float filteredAltitude = 0.0f;
float groundAltitude = 0.0f;
bool altitudeZeroed = false;
unsigned long lastBaroSampleMs = 0;
constexpr unsigned long BARO_SAMPLE_MS = 40;
constexpr float SEA_LEVEL_PRESSURE_PA = 101325.0f;

// -----------------------------------------------------------------------------
// Radio
// -----------------------------------------------------------------------------
RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
RadioPacket radioPacket{};
RadioPacket lastValidPacket{};
bool radioHasPacket = false;
bool failsafeActive = true;
unsigned long lastPacketMs = 0;
uint16_t lastFrameId = 0;

// -----------------------------------------------------------------------------
// Motor control
// -----------------------------------------------------------------------------
Servo motorFL, motorFR, motorRR, motorRL;

void writeMotors(int fl, int fr, int rr, int rl) {
  motorFL.writeMicroseconds(fl);
  motorFR.writeMicroseconds(fr);
  motorRR.writeMicroseconds(rr);
  motorRL.writeMicroseconds(rl);
}

void setAllMotors(int pulseWidthUs) {
  writeMotors(pulseWidthUs, pulseWidthUs, pulseWidthUs, pulseWidthUs);
}

// -----------------------------------------------------------------------------
// PID controller
// -----------------------------------------------------------------------------
struct PIDController {
  float kp;
  float ki;
  float kd;
  float integrator;
  float prevError;
  float integratorLimit;
  float outputLimit;
};

PIDController pitchPid{6.0f, 3.0f, 0.11f, 0.0f, 0.0f, 150.0f, PID_HEADROOM_US};
PIDController rollPid{6.0f, 3.0f, 0.11f, 0.0f, 0.0f, 150.0f, PID_HEADROOM_US};
PIDController yawPid{3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 120.0f};
PIDController altitudePid{2.8f, 1.2f, 0.8f, 0.0f, 0.0f, 200.0f, 150.0f};

void resetPid(PIDController &pid) {
  pid.integrator = 0.0f;
  pid.prevError = 0.0f;
}

float computePid(PIDController &pid, float setpoint, float measurement, float dt) {
  float error = setpoint - measurement;
  pid.integrator += error * pid.ki * dt;
  pid.integrator = constrain(pid.integrator, -pid.integratorLimit, pid.integratorLimit);

  float derivative = (dt > 0.0f) ? (error - pid.prevError) / dt : 0.0f;
  float output = (pid.kp * error) + pid.integrator + (pid.kd * derivative);
  pid.prevError = error;

  return constrain(output, -pid.outputLimit, pid.outputLimit);
}

// -----------------------------------------------------------------------------
// Arm/disarm management
// -----------------------------------------------------------------------------
bool armed = false;
bool altitudeHoldEnabled = false;
float altitudeTarget = 0.0f;
float altitudeHoldBaseline = 0.0f;
unsigned long armChangeMs = 0;

void buzzerSignal(uint8_t count, uint16_t onMs = 80, uint16_t offMs = 60) {
  for (uint8_t i = 0; i < count; ++i) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(onMs);
    digitalWrite(BUZZER_PIN, LOW);
    if (i + 1 < count) {
      delay(offMs);
    }
  }
}

void disarmMotors(const char *reason = nullptr) {
  if (armed) {
    buzzerSignal(2, 60, 60);
  }
  armed = false;
  altitudeHoldEnabled = false;
  altitudeTarget = 0.0f;
  altitudeHoldBaseline = 0.0f;
  resetPid(pitchPid);
  resetPid(rollPid);
  resetPid(yawPid);
  resetPid(altitudePid);
  setAllMotors(MOTOR_MIN_US);
  armChangeMs = millis();
#ifdef SERIAL_PORT_MONITOR
  if (reason) {
    Serial.print(F("Disarmed: "));
    Serial.println(reason);
  }
#endif
}

void tryArm() {
  if (armed) {
    return;
  }
  unsigned long now = millis();
  if (now - armChangeMs < 1000) {
    return;  // debounce
  }
  if (failsafeActive) {
    return;
  }
  if (lastValidPacket.throttle > 5) {
    return;
  }
  armed = true;
  resetPid(pitchPid);
  resetPid(rollPid);
  resetPid(yawPid);
  resetPid(altitudePid);
  altitudeTarget = filteredAltitude;
  altitudeHoldBaseline = filteredAltitude;
  buzzerSignal(1, 120, 80);
  armChangeMs = now;
#ifdef SERIAL_PORT_MONITOR
  Serial.println(F("Armed"));
#endif
}

// -----------------------------------------------------------------------------
// Sensor setup and processing
// -----------------------------------------------------------------------------
void calibrateImu() {
  constexpr size_t samples = 2000;
  long axSum = 0;
  long aySum = 0;
  long azSum = 0;
  long gxSum = 0;
  long gySum = 0;
  long gzSum = 0;

  for (size_t i = 0; i < samples; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    axSum += ax;
    aySum += ay;
    azSum += az;
    gxSum += gx;
    gySum += gy;
    gzSum += gz;
    delay(2);
  }

  accelOffset[0] = static_cast<float>(axSum) / samples;
  accelOffset[1] = static_cast<float>(aySum) / samples;
  accelOffset[2] = static_cast<float>(azSum) / samples - 16384.0f;  // subtract 1g

  gyroOffset[0] = static_cast<float>(gxSum) / samples;
  gyroOffset[1] = static_cast<float>(gySum) / samples;
  gyroOffset[2] = static_cast<float>(gzSum) / samples;
}

void updateImu(float dt) {
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float axf = (static_cast<float>(ax) - accelOffset[0]) / 16384.0f;
  float ayf = (static_cast<float>(ay) - accelOffset[1]) / 16384.0f;
  float azf = (static_cast<float>(az) - accelOffset[2]) / 16384.0f;

  angularRate.x = (static_cast<float>(gx) - gyroOffset[0]) / 131.0f;
  angularRate.y = (static_cast<float>(gy) - gyroOffset[1]) / 131.0f;
  angularRate.z = (static_cast<float>(gz) - gyroOffset[2]) / 131.0f;

  float rollAcc = atan2f(ayf, azf) * RAD_TO_DEG;
  float pitchAcc = atan2f(-axf, sqrtf(ayf * ayf + azf * azf)) * RAD_TO_DEG;

  const float alpha = 0.98f;
  attitude.roll = alpha * (attitude.roll + angularRate.x * dt) + (1.0f - alpha) * rollAcc;
  attitude.pitch = alpha * (attitude.pitch + angularRate.y * dt) + (1.0f - alpha) * pitchAcc;
  attitude.yaw += angularRate.z * dt;

  // Keep yaw between -180..180
  if (attitude.yaw > 180.0f) {
    attitude.yaw -= 360.0f;
  } else if (attitude.yaw < -180.0f) {
    attitude.yaw += 360.0f;
  }
}

void setupBarometer() {
  if (!barometer.begin()) {
    baroPresent = false;
    return;
  }
  baroPresent = true;
  constexpr size_t samples = 60;
  float altitudeSum = 0.0f;
  for (size_t i = 0; i < samples; ++i) {
    float tempC, pressurePa;
    if (barometer.read(tempC, pressurePa)) {
      float altitude = 44330.0f * (1.0f - powf(pressurePa / SEA_LEVEL_PRESSURE_PA, 0.19029495f));
      altitudeSum += altitude;
    }
    delay(20);
  }
  groundAltitude = altitudeSum / samples;
  filteredAltitude = 0.0f;
  altitudeZeroed = true;
}

void updateBarometer() {
  if (!baroPresent) {
    return;
  }
  unsigned long now = millis();
  if (now - lastBaroSampleMs < BARO_SAMPLE_MS) {
    return;
  }
  float tempC, pressurePa;
  if (barometer.read(tempC, pressurePa)) {
    float altitude = 44330.0f * (1.0f - powf(pressurePa / SEA_LEVEL_PRESSURE_PA, 0.19029495f));
    float relative = altitude - groundAltitude;
    const float beta = 0.85f;
    filteredAltitude = beta * filteredAltitude + (1.0f - beta) * relative;
  }
  lastBaroSampleMs = now;
}

// -----------------------------------------------------------------------------
// Radio processing
// -----------------------------------------------------------------------------
void setupRadio() {
  if (!radio.begin()) {
    while (true) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(150);
      digitalWrite(BUZZER_PIN, LOW);
      delay(150);
    }
  }
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76);
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);
  radio.openReadingPipe(1, kRadioAddress);
  radio.startListening();
}

void pollRadio() {
  bool gotPacket = false;
  while (radio.available()) {
    radio.read(&radioPacket, sizeof(radioPacket));
    gotPacket = true;
  }
  if (!gotPacket) {
    return;
  }
  if (!validatePacket(radioPacket)) {
    return;
  }
  failsafeActive = false;
  lastPacketMs = millis();
  radioHasPacket = true;
  if (radioPacket.frameId != lastFrameId) {
    lastFrameId = radioPacket.frameId;
  }
  lastValidPacket = radioPacket;
}

void updateFailsafe() {
  unsigned long now = millis();
  if (now - lastPacketMs > RADIO_TIMEOUT_MS) {
    if (!failsafeActive) {
      failsafeActive = true;
      disarmMotors("radio timeout");
    }
  }
}

// -----------------------------------------------------------------------------
// Flight control loop
// -----------------------------------------------------------------------------
unsigned long lastLoopMicros = 0;

void updateFlightControl(float dt) {
  if (!imuReady) {
    return;
  }
  if (failsafeActive || !armed) {
    unsigned long now = millis();
    if (now - armChangeMs > FAILSAFE_MOTOR_SPINDOWN_MS) {
      setAllMotors(MOTOR_MIN_US);
    } else {
      setAllMotors(MOTOR_IDLE_US);
    }
    return;
  }

  float pitchSet = (static_cast<float>(lastValidPacket.pitch) / 500.0f) * MAX_ANGLE_DEG;
  float rollSet = (static_cast<float>(lastValidPacket.roll) / 500.0f) * MAX_ANGLE_DEG;
  float yawRateSet = (static_cast<float>(lastValidPacket.yaw) / 500.0f) * MAX_YAW_RATE_DPS;

  float throttleUs = MOTOR_MIN_US + (static_cast<float>(lastValidPacket.throttle) * THROTTLE_SCALE);

  bool altHoldRequest = (lastValidPacket.switches & (1 << 1)) && baroPresent;
  if (altHoldRequest && !altitudeHoldEnabled) {
    altitudeHoldBaseline = filteredAltitude;
    altitudeTarget = filteredAltitude;
    resetPid(altitudePid);
  }
  altitudeHoldEnabled = altHoldRequest;

  if (altitudeHoldEnabled) {
    // Allow POT_1 to define a +/- range around the captured baseline altitude.
    float knob = static_cast<float>(lastValidPacket.knobs[0]) / 1023.0f;  // 0..1
    float offset = (knob - 0.5f) * (MAX_ALTITUDE_METERS * 2.0f);          // -range .. +range
    altitudeTarget = altitudeHoldBaseline + offset;
    float altitudeCorrection = computePid(altitudePid, altitudeTarget, filteredAltitude, dt);
    throttleUs += altitudeCorrection;
  } else {
    resetPid(altitudePid);
  }

  float pitchCorrection = computePid(pitchPid, pitchSet, attitude.pitch, dt);
  float rollCorrection = computePid(rollPid, rollSet, attitude.roll, dt);
  float yawCorrection = computePid(yawPid, yawRateSet, angularRate.z, dt);

  float mFL = throttleUs + pitchCorrection - rollCorrection + yawCorrection;
  float mFR = throttleUs + pitchCorrection + rollCorrection - yawCorrection;
  float mRR = throttleUs - pitchCorrection + rollCorrection + yawCorrection;
  float mRL = throttleUs - pitchCorrection - rollCorrection - yawCorrection;

  int outFL = constrain(static_cast<int>(mFL), MOTOR_MIN_US, MOTOR_MAX_US);
  int outFR = constrain(static_cast<int>(mFR), MOTOR_MIN_US, MOTOR_MAX_US);
  int outRR = constrain(static_cast<int>(mRR), MOTOR_MIN_US, MOTOR_MAX_US);
  int outRL = constrain(static_cast<int>(mRL), MOTOR_MIN_US, MOTOR_MAX_US);

  writeMotors(outFL, outFR, outRR, outRL);
}

// -----------------------------------------------------------------------------
// Status LED and buzzer
// -----------------------------------------------------------------------------
void updateStatusIndicators() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;

  unsigned long now = millis();
  if (!armed) {
    if (now - lastBlink >= 400) {
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
      lastBlink = now;
    }
  } else {
    digitalWrite(STATUS_LED_PIN, failsafeActive ? ((now / 200) % 2) : HIGH);
  }
}

// -----------------------------------------------------------------------------
// Setup and loop
// -----------------------------------------------------------------------------
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);

  Wire.begin();
  delay(100);

#ifdef SERIAL_PORT_MONITOR
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  Serial.println(F("Flight controller booting..."));
#endif

  imu.initialize();
  if (!imu.testConnection()) {
    buzzerSignal(4, 120, 80);
    while (true) {
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(200);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(200);
    }
  }

  calibrateImu();
  imuReady = true;

  setupBarometer();

  motorFL.attach(MOTOR_FL_PIN, MOTOR_MIN_US, MOTOR_MAX_US);
  motorFR.attach(MOTOR_FR_PIN, MOTOR_MIN_US, MOTOR_MAX_US);
  motorRR.attach(MOTOR_RR_PIN, MOTOR_MIN_US, MOTOR_MAX_US);
  motorRL.attach(MOTOR_RL_PIN, MOTOR_MIN_US, MOTOR_MAX_US);
  setAllMotors(MOTOR_MIN_US);

  setupRadio();
  buzzerSignal(3, 50, 80);

  lastLoopMicros = micros();
}

void loop() {
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastLoopMicros) * 1e-6f;
  if (dt <= 0.0f || dt > 0.02f) {
    dt = 0.004f;
  }
  lastLoopMicros = nowMicros;

  pollRadio();
  updateFailsafe();

  bool armSwitch = (lastValidPacket.switches & (1 << 0)) != 0;
  if (radioHasPacket) {
    if (!armSwitch && armed) {
      disarmMotors("arm switch low");
    } else if (armSwitch && !armed) {
      tryArm();
    }
  }

  updateImu(dt);
  updateBarometer();

  updateFlightControl(dt);
  updateStatusIndicators();

  unsigned long loopTime = micros() - nowMicros;
  if (loopTime < LOOP_INTERVAL_US) {
    delayMicroseconds(LOOP_INTERVAL_US - loopTime);
  }
}

