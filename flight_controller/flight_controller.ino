/*
 * Simple quadcopter flight controller for Arduino Nano
 *
 * Hardware:
 *  - Arduino Nano
 *  - NRF24L01 (CE=D4, CSN=D10)
 *  - MPU6050 IMU (INT=D2)
 *  - GY-63 MS5611 barometer (I2C)
 *  - ESC outputs:
 *      Front-Left  -> D3
 *      Front-Right -> D5
 *      Rear-Right  -> D6
 *      Rear-Left   -> D7
 *  - Buzzer -> D8
 *  - Status LED -> D13 (use a dedicated LED pin; D7 is already used by RL motor)
 *
 * Notes:
 *  - Requires the RF24 library from TMRh20 (https://github.com/nRF24/RF24).
 *  - Uses only Wire/SPI/Servo from the Arduino core to minimise extra dependencies.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <math.h>

// ---------------------------
// Pin assignments
// ---------------------------
const uint8_t PIN_NRF_CE = 4;
const uint8_t PIN_NRF_CSN = 10;

const uint8_t PIN_MOTOR_FL = 3;
const uint8_t PIN_MOTOR_FR = 5;
const uint8_t PIN_MOTOR_RR = 6;
const uint8_t PIN_MOTOR_RL = 7;

const uint8_t PIN_BUZZER = 8;
const uint8_t PIN_STATUS_LED = 13;

const uint8_t PIN_IMU_INT = 2;

// ---------------------------
// RF24 configuration
// ---------------------------
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const byte RADIO_ADDRESS[6] = "DRON1";

// ---------------------------
// Control packet definition
// ---------------------------
struct ControlPacket {
  uint8_t magic[2];   // 'D', 'R'
  uint8_t version;    // protocol version
  uint8_t sequence;   // rolling counter
  int16_t throttle;   // 0 .. 1000
  int16_t roll;       // -500 .. 500 (left/right stick)
  int16_t pitch;      // -500 .. 500 (forward/back)
  int16_t yaw;        // -500 .. 500 (rate command)
  int16_t aux1;       // -500 .. 500 (pot 1)
  int16_t aux2;       // -500 .. 500 (pot 2)
  uint8_t buttons;    // bit mask for buttons, joystick presses, toggles
  uint8_t reserved;   // future use (set to 0)
  uint16_t checksum;  // simple sum over bytes (excluding checksum field)
} __attribute__((packed));

// Button bit layout (matches RC transmitter)
enum ButtonBits : uint8_t {
  BTN_JOY_LEFT  = 0,
  BTN_JOY_RIGHT = 1,
  BTN_1         = 2,
  BTN_2         = 3,
  BTN_3         = 4,
  BTN_4         = 5,
  BTN_SW1       = 6,  // Arm/Disarm switch
  BTN_SW2       = 7   // Altitude hold switch
};

// ---------------------------
// Timing
// ---------------------------
const uint32_t LOOP_PERIOD_US = 4000;      // 250 Hz loop
const uint32_t SIGNAL_LOSS_TIMEOUT_US = 250000; // 250 ms before failsafe

// ---------------------------
// Motor output limits
// ---------------------------
const int PWM_MIN = 1000;
const int PWM_IDLE = 1070;
const int PWM_MAX = 2000;

// ---------------------------
// Control mapping
// ---------------------------
const float MAX_TILT_DEG = 35.0f;
const float MAX_YAW_RATE_DPS = 180.0f;

// ---------------------------
// PID controller
// ---------------------------
struct PidAxis {
  float kp;
  float ki;
  float kd;
  float integrator;
  float prevError;
  float outputLimit;

  void reset() {
    integrator = 0.0f;
    prevError = 0.0f;
  }

  float update(float error, float derivInput, float dt) {
    integrator += error * ki * dt;
    integrator = constrain(integrator, -outputLimit, outputLimit);

    float derivative = (error - prevError) / dt;
    // Blend derivative input with gyro measurement for better damping
    float output = kp * error + integrator + kd * (derivative - derivInput);

    prevError = error;
    return constrain(output, -outputLimit, outputLimit);
  }
};

PidAxis pidRoll  = {0.055f, 0.020f, 0.003f, 0.0f, 0.0f, 400.0f};
PidAxis pidPitch = {0.055f, 0.020f, 0.003f, 0.0f, 0.0f, 400.0f};
PidAxis pidYaw   = {0.080f, 0.020f, 0.000f, 0.0f, 0.0f, 400.0f};

// Altitude hold PID (acts on throttle)
PidAxis pidAltitude = {0.90f, 0.30f, 0.00f, 0.0f, 0.0f, 200.0f};

// ---------------------------
// IMU state
// ---------------------------
struct Vector3f {
  float x;
  float y;
  float z;
};

Vector3f gyroOffset = {0.0f, 0.0f, 0.0f};
Vector3f accelOffset = {0.0f, 0.0f, 0.0f};
Vector3f gyroDps = {0.0f, 0.0f, 0.0f};
Vector3f accelG = {0.0f, 0.0f, 0.0f};

float attitudeRollDeg = 0.0f;
float attitudePitchDeg = 0.0f;
float attitudeYawDeg = 0.0f;

float altitudeMetersRaw = 0.0f;
float altitudeMeters = 0.0f;
float altitudeVelocity = 0.0f;
float altitudeRef = 0.0f;

// ---------------------------
// MS5611 calibration data
// ---------------------------
const uint8_t MS5611_ADDR = 0x77;
uint16_t ms5611Cal[6] = {0};
bool ms5611Available = false;

// ---------------------------
// Control state
// ---------------------------
ControlPacket latestPacket = { {'D', 'R'}, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint32_t lastPacketMicros = 0;
bool radioHealthy = false;
bool failsafeActive = true;
bool isArmed = false;
bool altitudeHoldActive = false;

float throttleCommand = 0.0f;   // 0.0 .. 1.0
float rollCommandDeg = 0.0f;
float pitchCommandDeg = 0.0f;
float yawRateCommandDps = 0.0f;
float aux1Command = 0.0f;
float aux2Command = 0.0f;

// ---------------------------
// Motor outputs
// ---------------------------
Servo motorFL;
Servo motorFR;
Servo motorRR;
Servo motorRL;

// ---------------------------
// Utility helpers
// ---------------------------
template <typename T>
T constrainf(T value, T low, T high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

uint16_t computeChecksum(const ControlPacket &pkt) {
  const uint8_t *raw = reinterpret_cast<const uint8_t *>(&pkt);
  uint16_t sum = 0;
  for (size_t i = 0; i < sizeof(ControlPacket) - sizeof(pkt.checksum); ++i) {
    sum += raw[i];
  }
  return sum;
}

bool validatePacket(ControlPacket &pkt) {
  if (pkt.magic[0] != 'D' || pkt.magic[1] != 'R') {
    return false;
  }
  if (pkt.version != 1) {
    return false;
  }
  uint16_t calc = computeChecksum(pkt);
  return (calc == pkt.checksum);
}

float mapCommand(int16_t value, int16_t inMin, int16_t inMax, float outMin, float outMax) {
  value = constrain(value, inMin, inMax);
  float span = outMax - outMin;
  float normalized = static_cast<float>(value - inMin) / static_cast<float>(inMax - inMin);
  return outMin + normalized * span;
}

bool bitRead8(uint8_t value, uint8_t bitIndex) {
  return (value >> bitIndex) & 0x01;
}

void beepBuzzer(uint16_t frequency, uint16_t durationMs) {
  tone(PIN_BUZZER, frequency, durationMs);
}

void stopBuzzer() {
  noTone(PIN_BUZZER);
}

// ---------------------------
// I2C helpers
// ---------------------------
void i2cWriteByte(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void i2cWriteCommand(uint8_t address, uint8_t cmd) {
  Wire.beginTransmission(address);
  Wire.write(cmd);
  Wire.endTransmission();
}

uint8_t i2cReadByte(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)1);
  return Wire.read();
}

void i2cReadBytes(uint8_t address, uint8_t reg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, count);
  for (uint8_t i = 0; i < count; ++i) {
    dest[i] = Wire.read();
  }
}

// ---------------------------
// MPU6050 driver
// ---------------------------
const uint8_t MPU6050_ADDR = 0x68;

void initMPU6050() {
  Wire.begin();
  delay(10);

  i2cWriteByte(MPU6050_ADDR, 0x6B, 0x00); // Wake up
  i2cWriteByte(MPU6050_ADDR, 0x6C, 0x00);
  i2cWriteByte(MPU6050_ADDR, 0x1B, 0x08); // Gyro +/-500 deg/s
  i2cWriteByte(MPU6050_ADDR, 0x1C, 0x08); // Accel +/-4g
  i2cWriteByte(MPU6050_ADDR, 0x1A, 0x03); // DLPF 44Hz
  delay(100);

  // Calibration
  const int samples = 1000;
  Vector3f gyroSum = {0.0f, 0.0f, 0.0f};
  Vector3f accelSum = {0.0f, 0.0f, 0.0f};

  for (int i = 0; i < samples; ++i) {
    int16_t raw[7];
    uint8_t buffer[14];
    i2cReadBytes(MPU6050_ADDR, 0x3B, 14, buffer);
    for (uint8_t j = 0; j < 7; ++j) {
      raw[j] = (buffer[2 * j] << 8) | buffer[2 * j + 1];
    }

    accelSum.x += raw[0];
    accelSum.y += raw[1];
    accelSum.z += raw[2];

    gyroSum.x += raw[4];
    gyroSum.y += raw[5];
    gyroSum.z += raw[6];

    delay(2);
  }

  gyroOffset.x = gyroSum.x / samples;
  gyroOffset.y = gyroSum.y / samples;
  gyroOffset.z = gyroSum.z / samples;

  accelOffset.x = accelSum.x / samples;
  accelOffset.y = accelSum.y / samples;
  accelOffset.z = (accelSum.z / samples) - 16384.0f * cos(0.0f); // remove 1g
}

void readMPU6050() {
  uint8_t buffer[14];
  i2cReadBytes(MPU6050_ADDR, 0x3B, 14, buffer);

  int16_t rawAx = (buffer[0] << 8) | buffer[1];
  int16_t rawAy = (buffer[2] << 8) | buffer[3];
  int16_t rawAz = (buffer[4] << 8) | buffer[5];
  int16_t rawGx = (buffer[8] << 8) | buffer[9];
  int16_t rawGy = (buffer[10] << 8) | buffer[11];
  int16_t rawGz = (buffer[12] << 8) | buffer[13];

  // Sensitivity: gyro 65.5 LSB/deg/s (500 deg/s) ; accel 8192 LSB/g (4g)
  const float gyroScale = 1.0f / 65.5f;
  const float accelScale = 1.0f / 8192.0f;

  gyroDps.x = (rawGx - gyroOffset.x) * gyroScale;
  gyroDps.y = (rawGy - gyroOffset.y) * gyroScale;
  gyroDps.z = (rawGz - gyroOffset.z) * gyroScale;

  accelG.x = (rawAx - accelOffset.x) * accelScale;
  accelG.y = (rawAy - accelOffset.y) * accelScale;
  accelG.z = (rawAz - (accelOffset.z + 16384.0f)) * accelScale;
}

void updateAttitude(float dt) {
  readMPU6050();

  // Integrate gyro
  attitudeRollDeg += gyroDps.x * dt;
  attitudePitchDeg += gyroDps.y * dt;
  attitudeYawDeg += gyroDps.z * dt;

  // Accelerometer based angles
  float rollAcc = atan2f(accelG.y, accelG.z) * RAD_TO_DEG;
  float pitchAcc = atan2f(-accelG.x, sqrtf(accelG.y * accelG.y + accelG.z * accelG.z)) * RAD_TO_DEG;

  const float alpha = 0.98f;
  attitudeRollDeg = alpha * attitudeRollDeg + (1.0f - alpha) * rollAcc;
  attitudePitchDeg = alpha * attitudePitchDeg + (1.0f - alpha) * pitchAcc;

  // Keep yaw within [-180, 180]
  if (attitudeYawDeg > 180.0f) {
    attitudeYawDeg -= 360.0f;
  } else if (attitudeYawDeg < -180.0f) {
    attitudeYawDeg += 360.0f;
  }
}

// ---------------------------
// MS5611 driver
// ---------------------------
bool ms5611ReadPROM() {
  for (uint8_t i = 0; i < 6; ++i) {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(0xA2 + i * 2);
    if (Wire.endTransmission(false) != 0) {
      return false;
    }
    Wire.requestFrom(MS5611_ADDR, (uint8_t)2);
    if (Wire.available() < 2) {
      return false;
    }
    ms5611Cal[i] = (Wire.read() << 8) | Wire.read();
  }
  return true;
}

uint32_t ms5611ReadADC(uint8_t command) {
  i2cWriteCommand(MS5611_ADDR, command);
  delayMicroseconds(10000); // 10 ms for OSR=4096

  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) {
    return 0;
  }
  Wire.requestFrom(MS5611_ADDR, (uint8_t)3);
  if (Wire.available() < 3) {
    return 0;
  }
  uint32_t value = (uint32_t)Wire.read() << 16;
  value |= (uint32_t)Wire.read() << 8;
  value |= Wire.read();
  return value;
}

bool initMS5611() {
  ms5611Available = false;
  Wire.beginTransmission(MS5611_ADDR);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  ms5611Available = ms5611ReadPROM();
  return ms5611Available;
}

bool readMS5611(float &pressurePa, float &temperatureC) {
  if (!ms5611Available) {
    return false;
  }

  uint32_t D2 = ms5611ReadADC(0x58); // temperature
  uint32_t D1 = ms5611ReadADC(0x48); // pressure

  if (D1 == 0 || D2 == 0) {
    return false;
  }

  int32_t dT = (int32_t)D2 - ((int32_t)ms5611Cal[4] << 8);
  int64_t TEMP = 2000 + ((int64_t)dT * (int64_t)ms5611Cal[5]) / (1LL << 23);

  int64_t OFF = ((int64_t)ms5611Cal[1] << 16) + (((int64_t)ms5611Cal[3] * dT) >> 7);
  int64_t SENS = ((int64_t)ms5611Cal[0] << 15) + (((int64_t)ms5611Cal[2] * dT) >> 8);

  int32_t P = (((D1 * SENS) >> 21) - OFF) >> 15;

  temperatureC = TEMP / 100.0f;
  pressurePa = P / 100.0f * 100.0f; // convert mbar to Pa
  return true;
}

float pressureToAltitude(float pressurePa, float referencePressurePa) {
  return 44330.0f * (1.0f - powf(pressurePa / referencePressurePa, 0.1902949f));
}

void updateAltitude(float dt) {
  static float referencePressurePa = 101325.0f;
  static bool referenceInitialized = false;
  float pressurePa = 0.0f;
  float temperatureC = 0.0f;

  if (!ms5611Available) {
    return;
  }

  if (!readMS5611(pressurePa, temperatureC)) {
    return;
  }

  if (!referenceInitialized) {
    referencePressurePa = pressurePa;
    referenceInitialized = true;
    altitudeMeters = 0.0f;
    altitudeMetersRaw = 0.0f;
    altitudeVelocity = 0.0f;
  }

  altitudeMetersRaw = pressureToAltitude(pressurePa, referencePressurePa);
  const float alpha = 0.1f;
  float lastAltitude = altitudeMeters;
  altitudeMeters = altitudeMeters + alpha * (altitudeMetersRaw - altitudeMeters);
  altitudeVelocity = (altitudeMeters - lastAltitude) / dt;
}

// ---------------------------
// Radio handling
// ---------------------------
void initRadio() {
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(108);
  radio.setCRCLength(RF24_CRC_16);
  radio.openReadingPipe(0, RADIO_ADDRESS);
  radio.setPayloadSize(sizeof(ControlPacket));
  radio.startListening();
  radio.setRetries(5, 15);
}

bool fetchControlPacket() {
  bool received = false;
  while (radio.available()) {
    ControlPacket incoming;
    radio.read(&incoming, sizeof(ControlPacket));
    if (validatePacket(incoming)) {
      latestPacket = incoming;
      lastPacketMicros = micros();
      radioHealthy = true;
      received = true;
    }
  }
  return received;
}

void applyControlPacket() {
  failsafeActive = (micros() - lastPacketMicros) > SIGNAL_LOSS_TIMEOUT_US;
  if (failsafeActive) {
    throttleCommand = 0.0f;
    rollCommandDeg = 0.0f;
    pitchCommandDeg = 0.0f;
    yawRateCommandDps = 0.0f;
    aux1Command = 0.0f;
    aux2Command = 0.0f;
    altitudeHoldActive = false;
    return;
  }

  throttleCommand = mapCommand(latestPacket.throttle, 0, 1000, 0.0f, 1.0f);
  rollCommandDeg = mapCommand(latestPacket.roll, -500, 500, -MAX_TILT_DEG, MAX_TILT_DEG);
  pitchCommandDeg = mapCommand(latestPacket.pitch, -500, 500, -MAX_TILT_DEG, MAX_TILT_DEG);
  yawRateCommandDps = mapCommand(latestPacket.yaw, -500, 500, -MAX_YAW_RATE_DPS, MAX_YAW_RATE_DPS);
  aux1Command = mapCommand(latestPacket.aux1, -500, 500, -1.0f, 1.0f);
  aux2Command = mapCommand(latestPacket.aux2, -500, 500, -1.0f, 1.0f);

  bool switchArm = bitRead8(latestPacket.buttons, BTN_SW1);
  bool switchAltHold = bitRead8(latestPacket.buttons, BTN_SW2);

  if (!switchArm) {
    if (isArmed) {
      beepBuzzer(2000, 150);
    }
    isArmed = false;
    altitudeHoldActive = false;
  } else if (!isArmed && throttleCommand < 0.05f) {
    isArmed = true;
    beepBuzzer(2600, 120);
  }

  if (isArmed && switchAltHold) {
    if (!altitudeHoldActive) {
      altitudeRef = altitudeMeters;
      pidAltitude.reset();
      altitudeHoldActive = true;
    }
  } else {
    altitudeHoldActive = false;
  }
}

// ---------------------------
// Motor mixing
// ---------------------------
void writeMotorOutputs(float throttle, float rollCorr, float pitchCorr, float yawCorr) {
  float mixFL = throttle + pitchCorr + rollCorr - yawCorr;
  float mixFR = throttle + pitchCorr - rollCorr + yawCorr;
  float mixRR = throttle - pitchCorr - rollCorr - yawCorr;
  float mixRL = throttle - pitchCorr + rollCorr + yawCorr;

  mixFL = constrainf(mixFL, PWM_MIN, PWM_MAX);
  mixFR = constrainf(mixFR, PWM_MIN, PWM_MAX);
  mixRR = constrainf(mixRR, PWM_MIN, PWM_MAX);
  mixRL = constrainf(mixRL, PWM_MIN, PWM_MAX);

  motorFL.writeMicroseconds(static_cast<int>(mixFL));
  motorFR.writeMicroseconds(static_cast<int>(mixFR));
  motorRR.writeMicroseconds(static_cast<int>(mixRR));
  motorRL.writeMicroseconds(static_cast<int>(mixRL));
}

void updateMotorOutputs(float dt) {
  if (!isArmed || failsafeActive) {
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
    pidAltitude.reset();
    writeMotorOutputs(PWM_MIN, 0, 0, 0);
    return;
  }

  float desiredRoll = rollCommandDeg;
  float desiredPitch = pitchCommandDeg;
  float desiredYawRate = yawRateCommandDps;

  float errorRoll = desiredRoll - attitudeRollDeg;
  float errorPitch = desiredPitch - attitudePitchDeg;
  float errorYawRate = desiredYawRate - gyroDps.z;

  float rollOut = pidRoll.update(errorRoll, gyroDps.x, dt);
  float pitchOut = pidPitch.update(errorPitch, gyroDps.y, dt);
  float yawOut = pidYaw.update(errorYawRate, 0.0f, dt);

  float throttleUs = PWM_MIN + throttleCommand * (PWM_MAX - PWM_MIN);

  if (altitudeHoldActive) {
    float altitudeError = altitudeRef - altitudeMeters;
    float altitudeCorr = pidAltitude.update(altitudeError, altitudeVelocity, dt);
    throttleUs += altitudeCorr;
  }

  throttleUs = constrainf(throttleUs, PWM_IDLE, PWM_MAX);
  if (throttleCommand < 0.05f && !altitudeHoldActive) {
    throttleUs = PWM_MIN;
  }

  writeMotorOutputs(throttleUs, rollOut, pitchOut, yawOut);
}

// ---------------------------
// Status LED / Buzzer
// ---------------------------
void updateStatusIndicators() {
  static uint32_t lastToggle = 0;
  static bool ledState = false;
  uint32_t now = millis();

  uint16_t blinkInterval = 1000;
  if (failsafeActive || !isArmed) {
    blinkInterval = 500;
  } else if (altitudeHoldActive) {
    blinkInterval = 200;
  }

  if (now - lastToggle >= blinkInterval) {
    ledState = !ledState;
    digitalWrite(PIN_STATUS_LED, ledState ? HIGH : LOW);
    lastToggle = now;
  }

  if (failsafeActive && (now % 1500) < 200) {
    tone(PIN_BUZZER, 1500);
  } else if (!isArmed && (now % 2000) < 120) {
    tone(PIN_BUZZER, 1200);
  } else {
    noTone(PIN_BUZZER);
  }
}

// ---------------------------
// Setup
// ---------------------------
void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
  pinMode(PIN_BUZZER, OUTPUT);
  stopBuzzer();

  Serial.begin(115200);
  delay(200);
  Serial.println(F("Flight controller booting..."));

  Wire.begin();
  Wire.setClock(400000);

  initMPU6050();
  Serial.println(F("MPU6050 initialised"));

  ms5611Available = initMS5611();
  if (ms5611Available) {
    Serial.println(F("MS5611 detected"));
  } else {
    Serial.println(F("MS5611 not found - altitude hold disabled"));
  }

  initRadio();
  Serial.println(F("NRF24 receiver ready"));

  motorFL.attach(PIN_MOTOR_FL, PWM_MIN, PWM_MAX);
  motorFR.attach(PIN_MOTOR_FR, PWM_MIN, PWM_MAX);
  motorRR.attach(PIN_MOTOR_RR, PWM_MIN, PWM_MAX);
  motorRL.attach(PIN_MOTOR_RL, PWM_MIN, PWM_MAX);

  writeMotorOutputs(PWM_MIN, 0, 0, 0);
  beepBuzzer(2200, 100);
  delay(150);
  beepBuzzer(2500, 120);

  lastPacketMicros = micros();
  Serial.println(F("Calibration complete. Waiting for arm switch..."));
}

// ---------------------------
// Main loop
// ---------------------------
void loop() {
  static uint32_t lastLoop = micros();
  uint32_t now = micros();
  uint32_t elapsed = now - lastLoop;

  if (elapsed < LOOP_PERIOD_US) {
    return;
  }

  lastLoop = now;
  float dt = elapsed / 1000000.0f;

  fetchControlPacket();
  applyControlPacket();

  updateAttitude(dt);
  updateAltitude(dt);

  updateMotorOutputs(dt);
  updateStatusIndicators();
}
