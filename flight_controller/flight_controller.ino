#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <RF24.h>
#include <math.h>

// ----------------------- Pin assignments -----------------------
// Radio
constexpr uint8_t PIN_RF_CE = 4;     // CE on D4
constexpr uint8_t PIN_RF_CSN = 10;   // CSN on D10 (uses SPI hardware pins)

// Motors (ESC signal pins)
constexpr uint8_t PIN_MOTOR_FL = 3;  // Front-left
constexpr uint8_t PIN_MOTOR_FR = 5;  // Front-right
constexpr uint8_t PIN_MOTOR_RR = 6;  // Rear-right
constexpr uint8_t PIN_MOTOR_RL = 7;  // Rear-left

// Peripherals
constexpr uint8_t PIN_BUZZER = 8;
constexpr uint8_t PIN_STATUS_LED = 13; // Use built-in LED (D13) to avoid conflicts with motor pins

// ----------------------- Radio configuration -----------------------
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
const byte RADIO_PIPE[6] = "QFC01";

struct ControlPacket {
  uint16_t throttle; // 0-1023
  uint16_t roll;     // 0-1023, centred at 512
  uint16_t pitch;    // 0-1023, centred at 512
  uint16_t yaw;      // 0-1023, centred at 512
  uint8_t buttons;   // Bitfield for buttons D4-D7
  uint8_t switches;  // Bitfield: bit0 -> SW1, bit1 -> SW2
  uint16_t pot1;     // 0-1023
  uint16_t pot2;     // 0-1023
  uint32_t sequence; // Incrementing packet counter
};

ControlPacket rxPacket{};
unsigned long lastPacketMillis = 0;
bool failsafeActive = true;
unsigned long lastFailsafeTone = 0;

// ----------------------- Motor control -----------------------
Servo motorFL;
Servo motorFR;
Servo motorRR;
Servo motorRL;

constexpr int PWM_MIN = 1000;
constexpr int PWM_MAX = 2000;

void writeAllMotors(int pulse) {
  motorFL.writeMicroseconds(pulse);
  motorFR.writeMicroseconds(pulse);
  motorRR.writeMicroseconds(pulse);
  motorRL.writeMicroseconds(pulse);
}

// ----------------------- PID controller -----------------------
class PID {
public:
  PID(float kp, float ki, float kd, float iMin, float iMax)
    : kp(kp), ki(ki), kd(kd), iMin(iMin), iMax(iMax) {}

  void setTunings(float newKp, float newKi, float newKd) {
    kp = newKp;
    ki = newKi;
    kd = newKd;
  }

  void setIntegralLimits(float minVal, float maxVal) {
    iMin = minVal;
    iMax = maxVal;
  }

  float compute(float setpoint, float measured, float dt) {
    const float error = setpoint - measured;
    integral += error * dt;
    if (integral > iMax) integral = iMax;
    if (integral < iMin) integral = iMin;

    const float derivative = (dt > 0.0f) ? (error - previousError) / dt : 0.0f;
    previousError = error;

    return kp * error + ki * integral + kd * derivative;
  }

  void reset() {
    integral = 0.0f;
    previousError = 0.0f;
  }

private:
  float kp;
  float ki;
  float kd;
  float iMin;
  float iMax;
  float integral = 0.0f;
  float previousError = 0.0f;
};

// ----------------------- MPU6050 driver -----------------------
constexpr uint8_t MPU6050_ADDR = 0x68;

struct MpuCalibration {
  float accelOffsetX = 0.0f;
  float accelOffsetY = 0.0f;
  float accelOffsetZ = 0.0f;
  float gyroOffsetX = 0.0f;
  float gyroOffsetY = 0.0f;
  float gyroOffsetZ = 0.0f;
};

MpuCalibration mpuCal;

void mpuWriteByte(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void mpuReadBytes(uint8_t reg, uint8_t* buffer, uint8_t length) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, length, true);
  for (uint8_t i = 0; i < length && Wire.available(); ++i) {
    buffer[i] = Wire.read();
  }
}

bool initMpu() {
  mpuWriteByte(0x6B, 0x00); // Wake up device
  delay(100);
  mpuWriteByte(0x19, 0x07); // Sample rate divider
  mpuWriteByte(0x1A, 0x03); // DLPF 44 Hz
  mpuWriteByte(0x1B, 0x00); // Gyro ±250 deg/s
  mpuWriteByte(0x1C, 0x00); // Accel ±2 g
  return true;
}

void calibrateMpu() {
  constexpr int samples = 1000;
  long ax = 0, ay = 0, az = 0;
  long gx = 0, gy = 0, gz = 0;
  uint8_t raw[14];

  digitalWrite(PIN_STATUS_LED, HIGH);
  for (int i = 0; i < samples; ++i) {
    mpuReadBytes(0x3B, raw, 14);
    int16_t accelX = (raw[0] << 8) | raw[1];
    int16_t accelY = (raw[2] << 8) | raw[3];
    int16_t accelZ = (raw[4] << 8) | raw[5];
    int16_t gyroX = (raw[8] << 8) | raw[9];
    int16_t gyroY = (raw[10] << 8) | raw[11];
    int16_t gyroZ = (raw[12] << 8) | raw[13];

    ax += accelX;
    ay += accelY;
    az += accelZ;
    gx += gyroX;
    gy += gyroY;
    gz += gyroZ;
    delay(3);
  }
  digitalWrite(PIN_STATUS_LED, LOW);

  mpuCal.accelOffsetX = static_cast<float>(ax) / samples;
  mpuCal.accelOffsetY = static_cast<float>(ay) / samples;
  mpuCal.accelOffsetZ = static_cast<float>(az) / samples - 16384.0f; // Remove 1 g
  mpuCal.gyroOffsetX = static_cast<float>(gx) / samples;
  mpuCal.gyroOffsetY = static_cast<float>(gy) / samples;
  mpuCal.gyroOffsetZ = static_cast<float>(gz) / samples;
}

struct ImuSample {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

ImuSample readImu() {
  uint8_t raw[14];
  mpuReadBytes(0x3B, raw, 14);

  ImuSample sample{};
  sample.accelX = ((raw[0] << 8) | raw[1]) - mpuCal.accelOffsetX;
  sample.accelY = ((raw[2] << 8) | raw[3]) - mpuCal.accelOffsetY;
  sample.accelZ = ((raw[4] << 8) | raw[5]) - mpuCal.accelOffsetZ;
  sample.gyroX = ((raw[8] << 8) | raw[9]) - mpuCal.gyroOffsetX;
  sample.gyroY = ((raw[10] << 8) | raw[11]) - mpuCal.gyroOffsetY;
  sample.gyroZ = ((raw[12] << 8) | raw[13]) - mpuCal.gyroOffsetZ;

  const float accelScale = 16384.0f;
  const float gyroScale = 131.0f;

  sample.accelX /= accelScale;
  sample.accelY /= accelScale;
  sample.accelZ /= accelScale;
  sample.gyroX /= gyroScale;
  sample.gyroY /= gyroScale;
  sample.gyroZ /= gyroScale;

  return sample;
}

// ----------------------- Attitude estimation -----------------------
class AttitudeEstimator {
public:
  void update(const ImuSample& sample, float dt) {
    // Integrate gyro rates (deg/s) to get angles (deg)
    roll += sample.gyroX * dt;
    pitch += sample.gyroY * dt;
    yaw += sample.gyroZ * dt;

    // Calculate roll/pitch from accelerometer
    const float rollAccel = atan2f(sample.accelY, sample.accelZ) * RAD_TO_DEG;
    const float pitchAccel = atan2f(-sample.accelX,
                                    sqrtf(sample.accelY * sample.accelY + sample.accelZ * sample.accelZ)) * RAD_TO_DEG;

    const float alpha = 0.98f; // Complementary filter constant
    roll = alpha * roll + (1.0f - alpha) * rollAccel;
    pitch = alpha * pitch + (1.0f - alpha) * pitchAccel;
  }

  void reset(float initialRoll = 0.0f, float initialPitch = 0.0f, float initialYaw = 0.0f) {
    roll = initialRoll;
    pitch = initialPitch;
    yaw = initialYaw;
  }

  float getRoll() const { return roll; }
  float getPitch() const { return pitch; }
  float getYawRate() const { return yaw; } // Note: yaw is gyro integrated only, expect drift

private:
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
};

AttitudeEstimator attitude;

// ----------------------- MS5611 barometer driver -----------------------
constexpr uint8_t MS5611_ADDR = 0x77;

class MS5611 {
public:
  bool begin() {
    reset();
    delay(5);
    for (uint8_t i = 0; i < 6; ++i) {
      coeffs[i] = readPROM(0xA2 + i * 2);
    }
    return coeffs[0] != 0;
  }

  void reset() {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(0x1E);
    Wire.endTransmission();
  }

  bool read(float& temperatureC, float& pressurePa) {
    uint32_t d1 = readADC(0x48); // Pressure conversion OSR=4096
    uint32_t d2 = readADC(0x58); // Temperature conversion OSR=4096
    if (d1 == 0 || d2 == 0) {
      return false;
    }

    const int32_t dT = static_cast<int32_t>(d2) - static_cast<int32_t>(coeffs[4]) * 256;
    int64_t OFF = static_cast<int64_t>(coeffs[1]) * 65536 + (static_cast<int64_t>(coeffs[3]) * dT) / 128;
    int64_t SENS = static_cast<int64_t>(coeffs[0]) * 32768 + (static_cast<int64_t>(coeffs[2]) * dT) / 256;
    int32_t TEMP = 2000 + (dT * static_cast<int64_t>(coeffs[5])) / 8388608;

    // Second order temperature compensation
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    if (TEMP < 2000) {
      const int32_t tempMinus20 = TEMP - 2000;
      T2 = (dT * dT) >> 31;
      OFF2 = (5 * static_cast<int64_t>(tempMinus20) * static_cast<int64_t>(tempMinus20)) / 2;
      SENS2 = (5 * static_cast<int64_t>(tempMinus20) * static_cast<int64_t>(tempMinus20)) / 4;
      if (TEMP < -1500) {
        const int32_t tempPlus15 = TEMP + 1500;
        OFF2 += 7 * static_cast<int64_t>(tempPlus15) * static_cast<int64_t>(tempPlus15);
        SENS2 += (11 * static_cast<int64_t>(tempPlus15) * static_cast<int64_t>(tempPlus15)) / 2;
      }
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    int32_t P = (static_cast<int64_t>(d1) * SENS / 2097152 - OFF) / 32768;

    temperatureC = TEMP / 100.0f;
    pressurePa = P;
    return true;
  }

private:
  uint16_t coeffs[6] = {0};

  uint16_t readPROM(uint8_t reg) {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(MS5611_ADDR, (uint8_t)2);
    uint16_t value = (Wire.read() << 8) | Wire.read();
    return value;
  }

  uint32_t readADC(uint8_t command) {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(command);
    Wire.endTransmission();
    delayMicroseconds(10000); // Max conversion time for OSR=4096
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(MS5611_ADDR, (uint8_t)3);
    uint32_t value = 0;
    if (Wire.available() == 3) {
      value = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
    }
    return value;
  }
};

MS5611 barometer;
bool barometerReady = false;
float seaLevelPressurePa = 101325.0f;
float altitudeOffsetM = 0.0f;

float pressureToAltitude(float pressurePa) {
  return 44330.0f * (1.0f - powf(pressurePa / seaLevelPressurePa, 0.1903f));
}

// ----------------------- Control state -----------------------
PID pidRoll(4.0f, 0.0f, 0.045f, -100.0f, 100.0f);
PID pidPitch(4.0f, 0.0f, 0.045f, -100.0f, 100.0f);
PID pidYaw(2.0f, 0.0f, 0.02f, -50.0f, 50.0f);
PID pidAltitude(2.0f, 0.5f, 0.0f, -200.0f, 200.0f);

bool armed = false;
float altitudeTarget = 0.0f;
float lastAltitude = 0.0f;
unsigned long lastLoopMicros = 0;
unsigned long lastAltitudeUpdate = 0;
unsigned long statusBlinkTimer = 0;

void disarmMotors() {
  armed = false;
  pidRoll.reset();
  pidPitch.reset();
  pidYaw.reset();
  pidAltitude.reset();
  writeAllMotors(PWM_MIN);
}

// ----------------------- Radio handling -----------------------
void setupRadio() {
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(90);
  radio.setAutoAck(true);
  radio.setRetries(3, 5);
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, RADIO_PIPE);
  radio.startListening();
}

bool readRadio() {
  bool updated = false;
  while (radio.available()) {
    ControlPacket incoming;
    radio.read(&incoming, sizeof(incoming));
    rxPacket = incoming;
    lastPacketMillis = millis();
    failsafeActive = false;
    updated = true;
  }
  return updated;
}

bool isArmingSwitchEngaged() {
  return rxPacket.switches & 0x01;
}

bool isAltitudeHoldEnabled() {
  return rxPacket.switches & 0x02;
}

float normalizeStick(uint16_t value) {
  return (static_cast<int32_t>(value) - 512) / 512.0f;
}

float mapThrottle(uint16_t value) {
  const float clamped = constrain(value, 0, 1023);
  return (clamped / 1023.0f);
}

// ----------------------- Setup -----------------------
void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println(F("Quad Flight Controller Boot"));

  Wire.begin();
  Wire.setClock(400000);

  setupRadio();
  Serial.println(F("RF24 ready"));

  if (!initMpu()) {
    Serial.println(F("MPU6050 init failed"));
    tone(PIN_BUZZER, 400, 2000);
    while (true) {
      digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
      delay(200);
    }
  }
  Serial.println(F("Calibrating MPU6050... Keep the frame still."));
  calibrateMpu();
  Serial.println(F("MPU calibration complete"));

  barometerReady = barometer.begin();
  if (!barometerReady) {
    Serial.println(F("MS5611 init failed. Continuing without barometer."));
  } else {
    float tempC = 0.0f, pressurePa = 0.0f;
    if (barometer.read(tempC, pressurePa)) {
      seaLevelPressurePa = pressurePa;
      altitudeOffsetM = pressureToAltitude(pressurePa);
    }
    Serial.println(F("MS5611 ready"));
  }

  motorFL.attach(PIN_MOTOR_FL);
  motorFR.attach(PIN_MOTOR_FR);
  motorRR.attach(PIN_MOTOR_RR);
  motorRL.attach(PIN_MOTOR_RL);

  // Initialise ESCs with minimum signal
  writeAllMotors(PWM_MIN);
  delay(2000);

  lastLoopMicros = micros();
  lastPacketMillis = 0;
  statusBlinkTimer = millis();
}

// ----------------------- Main control loop -----------------------
void loop() {
  const unsigned long nowMicros = micros();
  float dt = (nowMicros - lastLoopMicros) / 1e6f;
  if (dt <= 0.0001f || dt > 0.02f) {
    dt = 0.005f; // Default to 200 Hz if timing is out of expected bounds
  }
  lastLoopMicros = nowMicros;

  readRadio();
  if (millis() - lastPacketMillis > 200) {
    failsafeActive = true;
  }

  if (failsafeActive) {
    if (armed) {
      disarmMotors();
    }
    const unsigned long now = millis();
    if (now - lastFailsafeTone > 500) {
      tone(PIN_BUZZER, 440, 80);
      lastFailsafeTone = now;
    }
  }

  const ImuSample imu = readImu();
  attitude.update(imu, dt);

  float altitude = lastAltitude;
  if (barometerReady) {
    const unsigned long now = millis();
    if (now - lastAltitudeUpdate > 20) { // 50 Hz
      float tempC = 0.0f, pressurePa = 0.0f;
      if (barometer.read(tempC, pressurePa) && pressurePa > 1000.0f) {
        altitude = pressureToAltitude(pressurePa) - altitudeOffsetM;
        lastAltitude = 0.9f * lastAltitude + 0.1f * altitude; // Low-pass filter
      }
      lastAltitudeUpdate = now;
    } else {
      altitude = lastAltitude;
    }
  }

  // Arming logic
  const bool armSwitch = isArmingSwitchEngaged();
  const float throttleNorm = mapThrottle(rxPacket.throttle);
  if (armSwitch && !armed && throttleNorm < 0.05f && !failsafeActive) {
    armed = true;
    altitudeTarget = lastAltitude;
    tone(PIN_BUZZER, 1047, 120);
  }
  if ((!armSwitch && armed) || failsafeActive) {
    disarmMotors();
  }

  // Status LED blink
  if (armed) {
    digitalWrite(PIN_STATUS_LED, HIGH);
  } else {
    if (millis() - statusBlinkTimer > 250) {
      digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
      statusBlinkTimer = millis();
    }
  }

  if (!armed) {
    return;
  }

  // Control computations
  const float rollCmd = normalizeStick(rxPacket.roll) * 35.0f;   // deg target
  const float pitchCmd = normalizeStick(rxPacket.pitch) * 35.0f; // deg target
  const float yawCmd = normalizeStick(rxPacket.yaw) * 180.0f;    // deg/s target

  const float rollCorrection = pidRoll.compute(rollCmd, attitude.getRoll(), dt);
  const float pitchCorrection = pidPitch.compute(pitchCmd, attitude.getPitch(), dt);
  const float yawCorrection = pidYaw.compute(yawCmd, imu.gyroZ, dt);

  float throttle = PWM_MIN + throttleNorm * (PWM_MAX - PWM_MIN);

  if (isAltitudeHoldEnabled()) {
    if (abs(normalizeStick(rxPacket.pitch)) < 0.05f &&
        abs(normalizeStick(rxPacket.roll)) < 0.05f &&
        abs(normalizeStick(rxPacket.yaw)) < 0.05f) {
      altitudeTarget += (static_cast<int32_t>(rxPacket.pot1) - 512) / 512.0f * 0.002f; // fine adjust
      altitudeTarget = constrain(altitudeTarget, -1.0f, 5.0f); // 5 m ceiling
    } else {
      altitudeTarget = lastAltitude;
    }
    const float altCorrection = pidAltitude.compute(altitudeTarget, lastAltitude, dt);
    throttle += altCorrection;
  }

  throttle = constrain(throttle, PWM_MIN, PWM_MAX);

  float motorFLPulse = throttle + pitchCorrection + rollCorrection - yawCorrection;
  float motorFRPulse = throttle + pitchCorrection - rollCorrection + yawCorrection;
  float motorRRPulse = throttle - pitchCorrection - rollCorrection - yawCorrection;
  float motorRLPulse = throttle - pitchCorrection + rollCorrection + yawCorrection;

  motorFLPulse = constrain(motorFLPulse, PWM_MIN, PWM_MAX);
  motorFRPulse = constrain(motorFRPulse, PWM_MIN, PWM_MAX);
  motorRRPulse = constrain(motorRRPulse, PWM_MIN, PWM_MAX);
  motorRLPulse = constrain(motorRLPulse, PWM_MIN, PWM_MAX);

  motorFL.writeMicroseconds(static_cast<int>(motorFLPulse));
  motorFR.writeMicroseconds(static_cast<int>(motorFRPulse));
  motorRR.writeMicroseconds(static_cast<int>(motorRRPulse));
  motorRL.writeMicroseconds(static_cast<int>(motorRLPulse));

  // Telemetry (optional)
  static unsigned long lastTelemetry = 0;
  if (millis() - lastTelemetry > 100) {
    Serial.print(F("Att:"));
    Serial.print(attitude.getRoll(), 2);
    Serial.print(',');
    Serial.print(attitude.getPitch(), 2);
    Serial.print(',');
    Serial.print(imu.gyroZ, 2);
    Serial.print(F(" Alt:"));
    Serial.print(lastAltitude, 2);
    Serial.print(F(" Thr:"));
    Serial.print(throttleNorm, 2);
    Serial.println();
    lastTelemetry = millis();
  }
}
