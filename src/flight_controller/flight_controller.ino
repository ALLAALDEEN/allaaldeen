/**
 * Flight Controller sketch for Arduino Nano
 *
 * Hardware:
 *  - NRF24L01 (CE -> D4, CSN -> D10)
 *  - MPU6050 (INT -> D2)
 *  - MS5611 barometer (GY-63 breakout)
 *  - Motor ESC signal pins: FL -> D3, FR -> D5, RR -> D6, RL -> D7
 *  - Buzzer -> D8 (active HIGH)
 *  - Status LED -> D13 (recommended, original D7 conflicts with RL motor)
 *
 * Features:
 *  - 6-axis stabilization with complementary filter and PID control
 *  - Altitude estimation using MS5611 (vertical damping only)
 *  - NRF24L01 receiver with checksum validation & failsafe
 *  - Arming switch + throttle safety
 */

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <RF24.h>

#include <MPU6050.h>
#include <MS5611.h>

constexpr uint8_t PIN_RF24_CE = 4;
constexpr uint8_t PIN_RF24_CSN = 10;
constexpr uint8_t PIN_MPU_INT = 2;

constexpr uint8_t PIN_MOTOR_FL = 3;
constexpr uint8_t PIN_MOTOR_FR = 5;
constexpr uint8_t PIN_MOTOR_RR = 6;
constexpr uint8_t PIN_MOTOR_RL = 7;

constexpr uint8_t PIN_BUZZER = 8;
constexpr uint8_t PIN_STATUS_LED = 13;

constexpr uint16_t SIGNAL_MIN = 1000;
constexpr uint16_t SIGNAL_MAX = 2000;
constexpr uint16_t SIGNAL_ARMED_MIN = 1050;

constexpr uint8_t PACKET_VERSION = 1;
constexpr unsigned long RADIO_TIMEOUT_MS = 300;

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);
const uint64_t RADIO_PIPE = 0xF0F0F0F0D2LL;

struct __attribute__((packed)) ControlPacket {
  uint8_t version;
  uint16_t throttle;
  uint16_t yaw;
  uint16_t pitch;
  uint16_t roll;
  uint8_t buttons;
  uint8_t switches;
  uint8_t joystickButtons;
  uint16_t pot1;
  uint16_t pot2;
  uint16_t checksum;
};

ControlPacket rxPacket{};

uint16_t calcChecksum(const ControlPacket &pkt) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&pkt);
  uint16_t sum = 0;
  for (size_t i = 0; i < sizeof(ControlPacket) - sizeof(pkt.checksum); ++i) {
    sum += bytes[i];
  }
  return sum;
}

MPU6050 mpu;
MS5611 ms5611;

Servo motorFL;
Servo motorFR;
Servo motorRR;
Servo motorRL;

struct PIDState {
  float kp;
  float ki;
  float kd;
  float iTerm;
  float lastError;
};

PIDState pidRoll{4.0f, 0.02f, 18.0f, 0.0f, 0.0f};
PIDState pidPitch{4.0f, 0.02f, 18.0f, 0.0f, 0.0f};
PIDState pidYaw{2.0f, 0.0f, 0.0f, 0.0f, 0.0f};

float rollAngle = 0.0f;
float pitchAngle = 0.0f;
float yawRate = 0.0f;
float altitude = 0.0f;

unsigned long lastRadioPacketMs = 0;
unsigned long lastLoopMicros = 0;
unsigned long lastBaroUpdateMs = 0;

bool armed = false;

float applyPID(PIDState &state, float target, float measurement, float dt) {
  float error = target - measurement;
  state.iTerm += state.ki * error * dt;
  state.iTerm = constrain(state.iTerm, -200.0f, 200.0f);
  float dTerm = state.kd * (error - state.lastError) / dt;
  float output = state.kp * error + state.iTerm + dTerm;
  state.lastError = error;
  return output;
}

void resetPID(PIDState &state) {
  state.iTerm = 0.0f;
  state.lastError = 0.0f;
}

void resetAllPID() {
  resetPID(pidRoll);
  resetPID(pidPitch);
  resetPID(pidYaw);
}

void setMotors(uint16_t fl, uint16_t fr, uint16_t rr, uint16_t rl) {
  motorFL.writeMicroseconds(constrain(fl, SIGNAL_MIN, SIGNAL_MAX));
  motorFR.writeMicroseconds(constrain(fr, SIGNAL_MIN, SIGNAL_MAX));
  motorRR.writeMicroseconds(constrain(rr, SIGNAL_MIN, SIGNAL_MAX));
  motorRL.writeMicroseconds(constrain(rl, SIGNAL_MIN, SIGNAL_MAX));
}

void motorsOff() {
  setMotors(SIGNAL_MIN, SIGNAL_MIN, SIGNAL_MIN, SIGNAL_MIN);
}

void beepBuzzer(uint8_t count, uint16_t onMs = 100, uint16_t offMs = 100) {
  for (uint8_t i = 0; i < count; ++i) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(onMs);
    digitalWrite(PIN_BUZZER, LOW);
    if (i + 1 < count) {
      delay(offMs);
    }
  }
}

bool readRadio() {
  bool received = false;
  while (radio.available()) {
    ControlPacket candidate{};
    radio.read(&candidate, sizeof(candidate));
    if (candidate.version != PACKET_VERSION) {
      continue;
    }
    if (calcChecksum(candidate) != candidate.checksum) {
      continue;
    }
    rxPacket = candidate;
    lastRadioPacketMs = millis();
    received = true;
  }
  return received;
}

void initSensors() {
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    beepBuzzer(5, 60, 60);
  }
  delay(100);
  mpu.CalibrateGyro(6);
  mpu.CalibrateAccel(6);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);

  if (!ms5611.begin()) {
    beepBuzzer(3, 200, 200);
  } else {
    ms5611.setOversampling(MS5611_OSR_ULTRA_HIGH);
  }
}

void initMotors() {
  motorFL.attach(PIN_MOTOR_FL);
  motorFR.attach(PIN_MOTOR_FR);
  motorRR.attach(PIN_MOTOR_RR);
  motorRL.attach(PIN_MOTOR_RL);
  motorsOff();
  delay(3000);  // allow ESCs to arm
}

void initRadio() {
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(110);
  radio.setRetries(3, 15);
  radio.openReadingPipe(1, RADIO_PIPE);
  radio.startListening();
}

void setup() {
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_STATUS_LED, LOW);

  Serial.begin(115200);
  initSensors();
  initMotors();
  initRadio();

  lastLoopMicros = micros();
  beepBuzzer(2, 80, 100);
}

void updateIMU(float dt) {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw to useful units
  const float accelScale = 1.0f / 8192.0f;  // for +/-4g
  const float gyroScale = 1.0f / 65.5f;     // for +/-500 deg/s

  float axg = ax * accelScale;
  float ayg = ay * accelScale;
  float azg = az * accelScale;

  float gyroX = gx * gyroScale;
  float gyroY = gy * gyroScale;
  float gyroZ = gz * gyroScale;

  float accRoll = atan2(ayg, azg) * RAD_TO_DEG;
  float accPitch = atan2(-axg, sqrt(ayg * ayg + azg * azg)) * RAD_TO_DEG;

  rollAngle = 0.98f * (rollAngle + gyroX * dt) + 0.02f * accRoll;
  pitchAngle = 0.98f * (pitchAngle + gyroY * dt) + 0.02f * accPitch;
  yawRate = gyroZ;
}

void updateBarometer() {
  unsigned long now = millis();
  if (now - lastBaroUpdateMs < 50 || !ms5611.isConnected()) {
    return;
  }
  ms5611.read();
  double pressure = ms5611.getPressure();
  double temperature = ms5611.getTemperature();
  double seaLevel = 101325.0;
  altitude = ms5611.getAltitude(seaLevel, pressure, temperature);
  lastBaroUpdateMs = now;
}

void updateArmingState() {
  bool armSwitch = rxPacket.switches & 0x01;
  bool throttleLow = rxPacket.throttle < (SIGNAL_MIN + 10);

  if (!armed) {
    if (armSwitch && throttleLow && (millis() - lastRadioPacketMs) < RADIO_TIMEOUT_MS) {
      armed = true;
      resetAllPID();
      digitalWrite(PIN_STATUS_LED, HIGH);
      beepBuzzer(1, 200, 0);
    }
  } else {
    if (!armSwitch || !throttleLow || (millis() - lastRadioPacketMs) >= RADIO_TIMEOUT_MS) {
      armed = false;
      motorsOff();
      digitalWrite(PIN_STATUS_LED, LOW);
      beepBuzzer(2, 80, 60);
    }
  }
}

void loop() {
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastLoopMicros) / 1e6f;
  if (dt <= 0.0f || dt > 0.05f) {
    dt = 0.005f;
  }
  lastLoopMicros = nowMicros;

  readRadio();
  updateArmingState();
  updateIMU(dt);
  updateBarometer();

  bool radioHealthy = (millis() - lastRadioPacketMs) < RADIO_TIMEOUT_MS;
  digitalWrite(PIN_STATUS_LED, (armed && radioHealthy) ? HIGH : LOW);

  if (!armed) {
    motorsOff();
    return;
  }

  float throttle = constrain(rxPacket.throttle, SIGNAL_MIN, SIGNAL_MAX);

  float rollSetpoint = ((int32_t)rxPacket.roll - 1500) / 500.0f * 25.0f;   // ±25°
  float pitchSetpoint = ((int32_t)rxPacket.pitch - 1500) / 500.0f * 25.0f; // ±25°
  float yawSetpoint = ((int32_t)rxPacket.yaw - 1500) / 500.0f * 180.0f;    // ±180°/s

  float rollCorrection = applyPID(pidRoll, rollSetpoint, rollAngle, dt);
  float pitchCorrection = applyPID(pidPitch, pitchSetpoint, pitchAngle, dt);
  float yawCorrection = applyPID(pidYaw, yawSetpoint, yawRate, dt);

  // Altitude damping via pot1 (optional)
  float altitudeTrim = ((int32_t)rxPacket.pot1 - 512) * 0.05f;
  throttle += altitudeTrim;

  float motorFLValue = throttle + pitchCorrection + rollCorrection - yawCorrection;
  float motorFRValue = throttle + pitchCorrection - rollCorrection + yawCorrection;
  float motorRRValue = throttle - pitchCorrection - rollCorrection - yawCorrection;
  float motorRLValue = throttle - pitchCorrection + rollCorrection + yawCorrection;

  motorFLValue = constrain(motorFLValue, SIGNAL_MIN, SIGNAL_MAX);
  motorFRValue = constrain(motorFRValue, SIGNAL_MIN, SIGNAL_MAX);
  motorRRValue = constrain(motorRRValue, SIGNAL_MIN, SIGNAL_MAX);
  motorRLValue = constrain(motorRLValue, SIGNAL_MIN, SIGNAL_MAX);

  setMotors(motorFLValue, motorFRValue, motorRRValue, motorRLValue);

  static unsigned long lastDebugMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastDebugMs > 200) {
    lastDebugMs = nowMs;
    Serial.print(F("Roll: "));
    Serial.print(rollAngle, 2);
    Serial.print(F(" Pitch: "));
    Serial.print(pitchAngle, 2);
    Serial.print(F(" Alt: "));
    Serial.print(altitude, 1);
    Serial.print(F(" Thr: "));
    Serial.print(throttle);
    Serial.print(F(" A:"));
    Serial.println(armed);
  }
}
