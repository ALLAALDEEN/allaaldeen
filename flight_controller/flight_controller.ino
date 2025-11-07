/*
  Flight Controller Firmware for Nano-based Quadcopter
  Hardware:
    - Arduino Nano
    - NRF24L01 (CE -> D4, CSN -> D10, SPI pins default)
    - MPU6050 (INT -> D2, SDA -> A4, SCL -> A5)
    - GY-36 MS5611 barometer (SDA -> A4, SCL -> A5)
    - Motors (ESC signal pins): FL -> D3, FR -> D5, RR -> D6, RL -> D7
    - Buzzer -> D8
    - Status LED -> D13 (change LED_PIN if you wire differently)

  Libraries required (install via Arduino Library Manager):
    - RF24 by TMRh20
    - Servo (built-in)
    - MPU6050 by Electronic Cats (or library providing getMotion6)
    - MS5611 by Rob Tillaart
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <MPU6050.h>
#include <MS5611.h>
#include <math.h>

// -------------------- Pin Configuration --------------------
constexpr uint8_t PIN_MOTOR_FL = 3;
constexpr uint8_t PIN_MOTOR_FR = 5;
constexpr uint8_t PIN_MOTOR_RR = 6;
constexpr uint8_t PIN_MOTOR_RL = 7;

constexpr uint8_t PIN_BUZZER = 8;
constexpr uint8_t PIN_LED = 13;  // Change if you keep LED on a different pin

constexpr uint8_t PIN_RF_CE = 4;
constexpr uint8_t PIN_RF_CSN = 10;

constexpr uint8_t PIN_IMU_INT = 2;

// -------------------- Radio Setup --------------------
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
const byte RADIO_ADDRESS_RX[6] = "CTRL1";
const byte RADIO_ADDRESS_TX[6] = "STAT1";

// -------------------- Sensor Objects --------------------
MPU6050 mpu;
MS5611 ms5611;

// -------------------- Motor Control --------------------
Servo motorFL;
Servo motorFR;
Servo motorRR;
Servo motorRL;

constexpr uint16_t ESC_MIN_US = 1000;
constexpr uint16_t ESC_MAX_US = 2000;
constexpr uint16_t ESC_ARM_US = 1050;
constexpr uint16_t ESC_IDLE_US = 1100;

// -------------------- Control Frame Definition --------------------
struct __attribute__((packed)) ControlFrame {
  uint16_t throttle;  // 1000-2000 us
  int16_t yaw;        // centi-degrees/sec setpoint (-500 .. 500)
  int16_t pitch;      // centi-degrees setpoint (-300 .. 300)
  int16_t roll;       // centi-degrees setpoint (-300 .. 300)
  uint8_t switches;   // bit0: SW1, bit1: SW2
  uint8_t buttons;    // bits 0-3 -> button1..4
  uint16_t pot1;      // raw 0-1023
  uint16_t pot2;      // raw 0-1023
  uint16_t checksum;
};

ControlFrame controlFrame{};

// -------------------- Telemetry Frame (optional back-channel) --------------------
struct __attribute__((packed)) TelemetryFrame {
  float altitudeMeters;
  float batteryVolts;
  float pitchDeg;
  float rollDeg;
  float yawDeg;
  uint16_t checksum;
};

TelemetryFrame telemetryFrame{};

// -------------------- IMU State --------------------
volatile bool imuDataReady = false;

float pitchAngle = 0.0f;
float rollAngle = 0.0f;
float yawAngle = 0.0f;

float gyroRates[3] = {0.0f, 0.0f, 0.0f};  // deg/s for X, Y, Z
float gyroOffsets[3] = {0.0f, 0.0f, 0.0f};
float accelOffsets[3] = {0.0f, 0.0f, 0.0f};

unsigned long lastLoopMicros = 0;

// -------------------- PID Controllers --------------------
class PIDController {
public:
  PIDController(float kp, float ki, float kd, float iLimit)
      : kp_(kp), ki_(ki), kd_(kd), integratorLimit_(iLimit) {}

  void setTunings(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void reset() {
    integrator_ = 0.0f;
    prevError_ = 0.0f;
    firstRun_ = true;
  }

  float update(float target, float measurement, float dt) {
    float error = target - measurement;
    integrator_ += error * dt * ki_;
    integrator_ = constrain(integrator_, -integratorLimit_, integratorLimit_);

    float derivative = 0.0f;
    if (!firstRun_) {
      derivative = (error - prevError_) / dt;
    } else {
      firstRun_ = false;
    }

    prevError_ = error;
    return kp_ * error + integrator_ + kd_ * derivative;
  }

private:
  float kp_;
  float ki_;
  float kd_;
  float integrator_ = 0.0f;
  float integratorLimit_;
  float prevError_ = 0.0f;
  bool firstRun_ = true;
};

PIDController pidRoll(3.5f, 1.2f, 0.04f, 100.0f);
PIDController pidPitch(3.5f, 1.2f, 0.04f, 100.0f);
PIDController pidYaw(2.0f, 0.8f, 0.0f, 80.0f);

// -------------------- Failsafe --------------------
unsigned long lastPacketMicros = 0;
constexpr unsigned long RADIO_TIMEOUT_US = 500000;  // 0.5 s
bool isArmed = false;

// -------------------- Utility Functions --------------------
uint16_t computeChecksum(const uint8_t* data, size_t length) {
  uint16_t sum = 0;
  for (size_t i = 0; i < length; ++i) {
    sum += data[i];
  }
  return 0xFFFFu - sum;
}

bool validateControlFrame(ControlFrame& frame) {
  uint16_t receivedChecksum = frame.checksum;
  frame.checksum = 0;
  uint16_t expectedChecksum = computeChecksum(reinterpret_cast<uint8_t*>(&frame), sizeof(ControlFrame));
  frame.checksum = receivedChecksum;
  return expectedChecksum == receivedChecksum;
}

void imuInterruptHandler() {
  imuDataReady = true;
}

void setupMotors() {
  motorFL.attach(PIN_MOTOR_FL, ESC_MIN_US, ESC_MAX_US);
  motorFR.attach(PIN_MOTOR_FR, ESC_MIN_US, ESC_MAX_US);
  motorRR.attach(PIN_MOTOR_RR, ESC_MIN_US, ESC_MAX_US);
  motorRL.attach(PIN_MOTOR_RL, ESC_MIN_US, ESC_MAX_US);

  motorFL.writeMicroseconds(ESC_ARM_US);
  motorFR.writeMicroseconds(ESC_ARM_US);
  motorRR.writeMicroseconds(ESC_ARM_US);
  motorRL.writeMicroseconds(ESC_ARM_US);
}

void writeMotors(float fl, float fr, float rr, float rl) {
  motorFL.writeMicroseconds(static_cast<uint16_t>(constrain(fl, ESC_MIN_US, ESC_MAX_US)));
  motorFR.writeMicroseconds(static_cast<uint16_t>(constrain(fr, ESC_MIN_US, ESC_MAX_US)));
  motorRR.writeMicroseconds(static_cast<uint16_t>(constrain(rr, ESC_MIN_US, ESC_MAX_US)));
  motorRL.writeMicroseconds(static_cast<uint16_t>(constrain(rl, ESC_MIN_US, ESC_MAX_US)));
}

void setBuzzer(bool enabled) {
  digitalWrite(PIN_BUZZER, enabled ? HIGH : LOW);
}

void signalStatus(bool ok) {
  digitalWrite(PIN_LED, ok ? HIGH : LOW);
}

void calibrateIMU() {
  constexpr uint16_t samples = 2000;
  float gyroSums[3] = {0.0f, 0.0f, 0.0f};
  float accelSums[3] = {0.0f, 0.0f, 0.0f};

  setBuzzer(true);
  for (uint16_t i = 0; i < samples; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gyroSums[0] += gx;
    gyroSums[1] += gy;
    gyroSums[2] += gz;

    accelSums[0] += ax;
    accelSums[1] += ay;
    accelSums[2] += az;
    delay(2);
  }
  setBuzzer(false);

  for (uint8_t i = 0; i < 3; ++i) {
    gyroOffsets[i] = gyroSums[i] / samples;
    accelOffsets[i] = accelSums[i] / samples;
  }

  // Align accelerometer offset for Z to remove gravity magnitude
  accelOffsets[2] -= 16384.0f;  // 1g in LSB at default sensitivity
}

void updateIMU(float dt) {
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  mpu.getMotion6(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw);

  float ax = (axRaw - accelOffsets[0]) / 16384.0f;
  float ay = (ayRaw - accelOffsets[1]) / 16384.0f;
  float az = (azRaw - accelOffsets[2]) / 16384.0f;

  float gx = (gxRaw - gyroOffsets[0]) / 131.0f;
  float gy = (gyRaw - gyroOffsets[1]) / 131.0f;
  float gz = (gzRaw - gyroOffsets[2]) / 131.0f;

  gyroRates[0] = gx;
  gyroRates[1] = gy;
  gyroRates[2] = gz;

  float accRoll = atan2f(ay, az) * RAD_TO_DEG;
  float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  constexpr float alpha = 0.98f;
  rollAngle = alpha * (rollAngle + gx * dt) + (1.0f - alpha) * accRoll;
  pitchAngle = alpha * (pitchAngle + gy * dt) + (1.0f - alpha) * accPitch;
  yawAngle += gz * dt;  // gyro integration; subject to drift

  // Normalize yaw to [-180, 180]
  if (yawAngle > 180.0f) yawAngle -= 360.0f;
  if (yawAngle < -180.0f) yawAngle += 360.0f;
}

float readAltitude() {
  ms5611.read();
  float pressure = ms5611.getPressure();  // hPa
  if (pressure <= 0.0f) {
    return 0.0f;
  }
  const float seaLevelPressure = 1013.25f;  // hPa
  return 44330.0f * (1.0f - pow(pressure / seaLevelPressure, 0.1903f));
}

void applyFailsafe() {
  writeMotors(ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US);
  setBuzzer(true);
  signalStatus(false);
  isArmed = false;
}

void disarmMotors() {
  writeMotors(ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US);
  isArmed = false;
  pidRoll.reset();
  pidPitch.reset();
  pidYaw.reset();
}

void handleArmLogic() {
  bool armSwitch = controlFrame.switches & 0x01;
  if (armSwitch && !isArmed && controlFrame.throttle < (ESC_MIN_US + 20)) {
    isArmed = true;
    setBuzzer(false);
    signalStatus(true);
  } else if (!armSwitch && isArmed) {
    disarmMotors();
    setBuzzer(true);
    delay(200);
    setBuzzer(false);
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  setBuzzer(false);
  signalStatus(false);

  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  if (!mpu.testConnection()) {
    setBuzzer(true);
    while (true) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(200);
    }
  }
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  mpu.setRate(4);  // ~200 Hz

  if (!ms5611.begin()) {
    setBuzzer(true);
    while (true) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(100);
    }
  }
  ms5611.setOversampling(MS5611_OSR_4096);

  pinMode(PIN_IMU_INT, INPUT);
  mpu.setIntDataReadyEnabled(true);
  attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), imuInterruptHandler, RISING);
  imuDataReady = true;  // allow loop to run before first interrupt

  setupMotors();
  calibrateIMU();

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(115);
  radio.enableDynamicPayloads();
  radio.setRetries(5, 15);
  radio.openReadingPipe(1, RADIO_ADDRESS_RX);
  radio.openWritingPipe(RADIO_ADDRESS_TX);
  radio.startListening();

  lastLoopMicros = micros();
  lastPacketMicros = micros();
  setBuzzer(false);
  signalStatus(true);
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastLoopMicros) / 1e6f;
  lastLoopMicros = now;
  if (dt <= 0.0f || dt > 0.05f) {
    dt = 0.005f;  // default to 5 ms if timing glitch
  }

  if (radio.available()) {
    ControlFrame incoming{};
    radio.read(&incoming, sizeof(ControlFrame));
    if (validateControlFrame(incoming)) {
      controlFrame = incoming;
      lastPacketMicros = now;
      handleArmLogic();
    }
  }

  if ((now - lastPacketMicros) > RADIO_TIMEOUT_US) {
    applyFailsafe();
    return;
  }

  if (!imuDataReady) {
    // No fresh IMU data yet; keep ESCs at idle when armed
    if (!isArmed) {
      writeMotors(ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US);
    } else {
      writeMotors(ESC_IDLE_US, ESC_IDLE_US, ESC_IDLE_US, ESC_IDLE_US);
    }
    return;
  }

  imuDataReady = false;
  updateIMU(dt);

  float setpointPitch = controlFrame.pitch / 100.0f;  // convert centi-degrees to degrees
  float setpointRoll = controlFrame.roll / 100.0f;
  float setpointYawRate = controlFrame.yaw / 100.0f;   // deg/s

  float throttle = static_cast<float>(controlFrame.throttle);

  float rollCorrection = pidRoll.update(setpointRoll, rollAngle, dt);
  float pitchCorrection = pidPitch.update(setpointPitch, pitchAngle, dt);
  float yawCorrection = pidYaw.update(setpointYawRate, gyroRates[2], dt);

  float motorFLus = throttle + pitchCorrection + rollCorrection - yawCorrection;
  float motorFRus = throttle + pitchCorrection - rollCorrection + yawCorrection;
  float motorRRus = throttle - pitchCorrection - rollCorrection - yawCorrection;
  float motorRLus = throttle - pitchCorrection + rollCorrection + yawCorrection;

  if (!isArmed) {
    disarmMotors();
  } else {
    writeMotors(motorFLus, motorFRus, motorRRus, motorRLus);
  }

  static uint32_t telemetryTimer = 0;
  if (millis() - telemetryTimer > 100) {
    telemetryTimer = millis();
    telemetryFrame.altitudeMeters = readAltitude();
    telemetryFrame.pitchDeg = pitchAngle;
    telemetryFrame.rollDeg = rollAngle;
    telemetryFrame.yawDeg = yawAngle;
    telemetryFrame.batteryVolts = controlFrame.pot1 * (5.0f / 1023.0f) * 3.0f;  // placeholder scaling
    telemetryFrame.checksum = 0;
    telemetryFrame.checksum =
        computeChecksum(reinterpret_cast<uint8_t*>(&telemetryFrame), sizeof(TelemetryFrame));

    radio.stopListening();
    radio.write(&telemetryFrame, sizeof(TelemetryFrame));
    radio.startListening();
  }
}

