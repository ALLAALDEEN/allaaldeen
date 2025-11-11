#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <EEPROM.h>
#include <MPU6050.h>

#include "../shared/comm_protocol.h"

using namespace drone;

// ---------------------------------------------------------------------------
// Pin assignments (Arduino Nano)
// ---------------------------------------------------------------------------

constexpr uint8_t PIN_MOTOR_FL = 3;   // Front left
constexpr uint8_t PIN_MOTOR_FR = 5;   // Front right
constexpr uint8_t PIN_MOTOR_RL = 6;   // Rear left
constexpr uint8_t PIN_MOTOR_RR = 9;   // Rear right

constexpr uint8_t PIN_BUZZER = 4;
constexpr uint8_t PIN_STATUS_LED = 13;
constexpr uint8_t PIN_BATTERY_SENSE = A7; // Voltage divider input

constexpr uint8_t RADIO_PIN_CE = 7;
constexpr uint8_t RADIO_PIN_CSN = 8;

// ---------------------------------------------------------------------------
// Configuration constants
// ---------------------------------------------------------------------------

constexpr uint16_t ESC_MIN_US = 1000;
constexpr uint16_t ESC_MAX_US = 2000;
constexpr uint16_t ESC_IDLE_SPIN_US = 1120;

constexpr float GYRO_SCALE = 1.0f / 131.0f;    // MPU6050 default sensitivity (deg/s)
constexpr float ACCEL_SCALE = 1.0f / 16384.0f; // MPU6050 default (g)

constexpr float LOOP_TARGET_HZ = 400.0f;
constexpr float LOOP_DT = 1.0f / LOOP_TARGET_HZ;

constexpr uint32_t LINK_TIMEOUT_US = 150000; // 150 ms

constexpr uint8_t EEPROM_SIGNATURE = 0x42;

// PID defaults
constexpr float PID_ROLL_P = 4.0f;
constexpr float PID_ROLL_I = 0.04f;
constexpr float PID_ROLL_D = 16.0f;

constexpr float PID_PITCH_P = 4.0f;
constexpr float PID_PITCH_I = 0.04f;
constexpr float PID_PITCH_D = 16.0f;

constexpr float PID_YAW_P = 3.0f;
constexpr float PID_YAW_I = 0.03f;
constexpr float PID_YAW_D = 0.0f;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

enum class SystemState : uint8_t
{
    WAIT_RADIO = 0,
    CONFIRM_KILL_SWITCH,
    WAIT_CALIBRATION_REQUEST,
    CALIBRATING,
    WAIT_ARM,
    WAIT_IDLE_SPIN,
    READY,
    FAILSAFE
};

struct CalibrationData
{
    int16_t accelOffset[3] = {0, 0, 0};
    int16_t gyroOffset[3] = {0, 0, 0};
    uint16_t escMin = ESC_MIN_US;
    uint16_t escMax = ESC_MAX_US;
    uint8_t signature = EEPROM_SIGNATURE;
};

struct AxisState
{
    float angle = 0.0f;
    float rate = 0.0f;
};

struct PID
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral = 0.0f;
    float previousError = 0.0f;
    float outputMin = -400.0f;
    float outputMax = 400.0f;

    float update(float target, float measurement, float rate, float dt)
    {
        const float error = target - measurement;
        integral += error * ki * dt;
        integral = constrain(integral, outputMin, outputMax);
        const float derivative = -rate;
        float output = (kp * error) + integral + (kd * derivative);
        output = constrain(output, outputMin, outputMax);
        previousError = error;
        return output;
    }

    void reset()
    {
        integral = 0.0f;
        previousError = 0.0f;
    }
};

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

RF24 radio(RADIO_PIN_CE, RADIO_PIN_CSN);
MPU6050 mpu;
Servo motors[4];

CalibrationData g_calibration;
ControlFrame g_lastControl{};
TelemetryFrame g_telemetry{};

AxisState g_roll{};
AxisState g_pitch{};
AxisState g_yaw{};

PID g_pidRoll{PID_ROLL_P, PID_ROLL_I, PID_ROLL_D};
PID g_pidPitch{PID_PITCH_P, PID_PITCH_I, PID_PITCH_D};
PID g_pidYaw{PID_YAW_P, PID_YAW_I, PID_YAW_D};

SystemState g_state = SystemState::WAIT_RADIO;

uint32_t g_lastLoopMicros = 0;
uint32_t g_lastControlMicros = 0;
uint32_t g_lastTelemetryMillis = 0;

bool g_linkEstablished = false;
bool g_calibrationComplete = false;
bool g_motorsArmed = false;
bool g_motorsSpinning = false;
bool g_escCalibrated = false;

uint8_t g_prevActionFlags = ACTION_NONE;
uint8_t g_radioSequence = 0;

float g_verticalVelocity = 0.0f;
float g_altitudeCm = 0.0f;

// ---------------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------------

void blinkStatus(uint8_t times, uint16_t onMs = 60, uint16_t offMs = 60)
{
    for (uint8_t i = 0; i < times; ++i)
    {
        digitalWrite(PIN_STATUS_LED, HIGH);
        delay(onMs);
        digitalWrite(PIN_STATUS_LED, LOW);
        delay(offMs);
    }
}

void beep(uint16_t frequency, uint16_t durationMs)
{
    tone(PIN_BUZZER, frequency, durationMs);
    delay(durationMs + 10);
    noTone(PIN_BUZZER);
}

void beepSequence(std::initializer_list<uint16_t> tones, uint16_t durationMs = 80, uint16_t pauseMs = 40)
{
    for (uint16_t freq : tones)
    {
        tone(PIN_BUZZER, freq, durationMs);
        delay(durationMs + pauseMs);
        noTone(PIN_BUZZER);
    }
}

void saveCalibration()
{
    EEPROM.put(0, g_calibration);
}

void loadCalibration()
{
    EEPROM.get(0, g_calibration);
    if (g_calibration.signature != EEPROM_SIGNATURE)
    {
        g_calibration = CalibrationData{};
        saveCalibration();
    }
    else
    {
        g_escCalibrated = true;
        g_calibrationComplete = true; // IMU offsets are stored
    }
}

float analogToBatteryVoltage(uint16_t raw)
{
    // Assume 11:1 divider (e.g., 100k / 10k) and 5V reference.
    constexpr float VREF = 5.0f;
    constexpr float DIVIDER_RATIO = 11.0f;
    return (static_cast<float>(raw) * VREF / 1023.0f) * DIVIDER_RATIO;
}

// ---------------------------------------------------------------------------
// ESC & IMU calibration
// ---------------------------------------------------------------------------

void writeAllMotors(uint16_t microseconds)
{
    for (Servo &motor : motors)
    {
        motor.writeMicroseconds(microseconds);
    }
}

void calibrateEscs()
{
    g_state = SystemState::CALIBRATING;
    g_telemetry.status_flags |= STATUS_CALIBRATING;
    beepSequence({2000, 2000, 2000}, 120, 60);

    writeAllMotors(ESC_MAX_US);
    delay(2500);
    writeAllMotors(ESC_MIN_US);
    delay(2500);

    g_calibration.escMin = ESC_MIN_US;
    g_calibration.escMax = ESC_MAX_US;
    g_escCalibrated = true;

    saveCalibration();
    g_state = SystemState::WAIT_ARM;
    g_telemetry.status_flags &= ~STATUS_CALIBRATING;
    beepSequence({1500, 1800, 2100}, 80, 40);
    blinkStatus(3);
}

void calibrateImu(uint16_t samples = 2000)
{
    g_state = SystemState::CALIBRATING;
    g_telemetry.status_flags |= STATUS_CALIBRATING;
    beepSequence({1600, 1600}, 100, 40);

    int64_t accelSums[3] = {0, 0, 0};
    int64_t gyroSums[3] = {0, 0, 0};

    for (uint16_t i = 0; i < samples; ++i)
    {
        mpu.getMotion6(&accelSums[0], &accelSums[1], &accelSums[2],
                       &gyroSums[0], &gyroSums[1], &gyroSums[2]);
        delayMicroseconds(1000);
    }

    g_calibration.accelOffset[0] = accelSums[0] / static_cast<int32_t>(samples);
    g_calibration.accelOffset[1] = accelSums[1] / static_cast<int32_t>(samples);
    g_calibration.accelOffset[2] = (accelSums[2] / static_cast<int32_t>(samples)) - 16384;

    g_calibration.gyroOffset[0] = gyroSums[0] / static_cast<int32_t>(samples);
    g_calibration.gyroOffset[1] = gyroSums[1] / static_cast<int32_t>(samples);
    g_calibration.gyroOffset[2] = gyroSums[2] / static_cast<int32_t>(samples);

    mpu.setXAccelOffset(g_calibration.accelOffset[0]);
    mpu.setYAccelOffset(g_calibration.accelOffset[1]);
    mpu.setZAccelOffset(g_calibration.accelOffset[2]);

    mpu.setXGyroOffset(g_calibration.gyroOffset[0]);
    mpu.setYGyroOffset(g_calibration.gyroOffset[1]);
    mpu.setZGyroOffset(g_calibration.gyroOffset[2]);

    g_calibrationComplete = true;
    saveCalibration();

    g_state = SystemState::WAIT_ARM;
    g_telemetry.status_flags &= ~STATUS_CALIBRATING;
    beepSequence({1300, 1700, 2000}, 70, 40);
    blinkStatus(2);
}

// ---------------------------------------------------------------------------
// Radio handling
// ---------------------------------------------------------------------------

void sendTelemetry()
{
    g_telemetry.roll = static_cast<int16_t>(g_roll.angle * 100.0f);
    g_telemetry.pitch = static_cast<int16_t>(g_pitch.angle * 100.0f);
    g_telemetry.yaw = static_cast<int16_t>(g_yaw.angle * 100.0f);
    g_telemetry.vertical_velocity = static_cast<int16_t>(g_verticalVelocity * 100.0f);
    g_telemetry.throttle = g_lastControl.throttle;
    g_telemetry.altitude_cm = static_cast<uint16_t>(max(0.0f, g_altitudeCm));

    const float batteryVoltage = analogToBatteryVoltage(analogRead(PIN_BATTERY_SENSE));
    g_telemetry.battery_mv = static_cast<uint16_t>(batteryVoltage * 1000.0f);

    g_telemetry.status_flags = STATUS_CONNECTED;
    if (g_motorsArmed)
    {
        g_telemetry.status_flags |= STATUS_ARMED;
    }
    if (g_motorsSpinning)
    {
        g_telemetry.status_flags |= STATUS_MOTORS_SPINNING;
    }
    if (g_state == SystemState::CALIBRATING)
    {
        g_telemetry.status_flags |= STATUS_CALIBRATING;
    }
    if (g_state == SystemState::FAILSAFE)
    {
        g_telemetry.status_flags |= STATUS_FAILSAFE;
    }
    if (g_escCalibrated)
    {
        g_telemetry.status_flags |= STATUS_ESC_CALIBRATED;
    }
    if (g_calibrationComplete)
    {
        g_telemetry.status_flags |= STATUS_IMU_READY;
    }

    switch (g_state)
    {
    case SystemState::WAIT_RADIO:
        g_telemetry.guide_phase = GUIDE_RADIO_LINK;
        break;
    case SystemState::CONFIRM_KILL_SWITCH:
        g_telemetry.guide_phase = GUIDE_SWITCH_KILL;
        break;
    case SystemState::WAIT_CALIBRATION_REQUEST:
        g_telemetry.guide_phase = GUIDE_CALIBRATE_PROMPT;
        break;
    case SystemState::CALIBRATING:
        g_telemetry.guide_phase = GUIDE_CALIBRATING;
        break;
    case SystemState::WAIT_ARM:
        g_telemetry.guide_phase = GUIDE_ARM_PROMPT;
        break;
    case SystemState::WAIT_IDLE_SPIN:
        g_telemetry.guide_phase = GUIDE_IDLE_SPIN_PROMPT;
        break;
    case SystemState::READY:
        g_telemetry.guide_phase = GUIDE_READY_TO_FLY;
        break;
    case SystemState::FAILSAFE:
    default:
        g_telemetry.guide_phase = GUIDE_RADIO_LINK;
        break;
    }

    g_telemetry.rf_channel = RADIO_CHANNEL;
    g_telemetry.sequence++;
    finalizeFrame(g_telemetry);
    radio.writeAckPayload(1, &g_telemetry, sizeof(g_telemetry));
}

void processControlFrame(const ControlFrame &frame)
{
    g_lastControl = frame;
    g_lastControlMicros = micros();

    if (!g_linkEstablished)
    {
        g_linkEstablished = true;
        g_state = SystemState::CONFIRM_KILL_SWITCH;
        beepSequence({1000, 1500, 2000}, 80, 50);
    }

    const uint8_t risingFlags = (frame.action_flags & ~g_prevActionFlags);
    g_prevActionFlags = frame.action_flags;

    if (frame.arm_mode == ARM_MODE_KILL)
    {
        if (g_motorsArmed || g_motorsSpinning)
        {
            beepSequence({2000, 1200}, 80, 40);
        }
        g_motorsArmed = false;
        g_motorsSpinning = false;
        g_pidRoll.reset();
        g_pidPitch.reset();
        g_pidYaw.reset();
        if (g_linkEstablished)
        {
            g_state = SystemState::WAIT_ARM;
        }
    }
    else if (g_state == SystemState::WAIT_ARM && g_calibrationComplete && g_escCalibrated)
    {
        if (!g_motorsArmed && frame.throttle <= (ESC_MIN_US + 20))
        {
            g_motorsArmed = true;
            beepSequence({1500, 2100}, 70, 30);
            g_state = SystemState::WAIT_IDLE_SPIN;
        }
    }

    if (risingFlags & ACTION_REQUEST_CALIBRATION)
    {
        if (g_state == SystemState::WAIT_CALIBRATION_REQUEST || g_state == SystemState::WAIT_ARM)
        {
            calibrateImu();
            calibrateEscs();
        }
    }

    if (risingFlags & ACTION_REQUEST_IDLE_SPIN)
    {
        if (g_motorsArmed)
        {
            g_motorsSpinning = true;
            beepSequence({1800, 2200}, 70, 40);
            if (g_state == SystemState::WAIT_IDLE_SPIN)
            {
                g_state = SystemState::READY;
            }
        }
    }

    if (g_state == SystemState::CONFIRM_KILL_SWITCH && frame.arm_mode == ARM_MODE_KILL)
    {
        g_state = SystemState::WAIT_CALIBRATION_REQUEST;
    }
}

void pollRadio()
{
    if (radio.available())
    {
        ControlFrame frame{};
        radio.read(&frame, sizeof(frame));
        if (validateFrame(frame))
        {
            processControlFrame(frame);
            sendTelemetry();
        }
    }

    const uint32_t nowMicros = micros();
    if (g_linkEstablished && (nowMicros - g_lastControlMicros) > LINK_TIMEOUT_US)
    {
        g_state = SystemState::FAILSAFE;
        g_motorsArmed = false;
        g_motorsSpinning = false;
    }
    else if (g_state == SystemState::FAILSAFE && (nowMicros - g_lastControlMicros) <= LINK_TIMEOUT_US)
    {
        g_state = SystemState::WAIT_ARM;
    }
}

// ---------------------------------------------------------------------------
// Sensor processing
// ---------------------------------------------------------------------------

void updateImu(float dt)
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    if (!mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz))
    {
        return;
    }

    ax -= g_calibration.accelOffset[0];
    ay -= g_calibration.accelOffset[1];
    az -= g_calibration.accelOffset[2];

    gx -= g_calibration.gyroOffset[0];
    gy -= g_calibration.gyroOffset[1];
    gz -= g_calibration.gyroOffset[2];

    const float gyroX = gx * GYRO_SCALE;
    const float gyroY = gy * GYRO_SCALE;
    const float gyroZ = gz * GYRO_SCALE;

    g_roll.rate = gyroX;
    g_pitch.rate = gyroY;
    g_yaw.rate = gyroZ;

    g_roll.angle += gyroX * dt;
    g_pitch.angle += gyroY * dt;
    g_yaw.angle += gyroZ * dt;

    const float accelX = ax * ACCEL_SCALE;
    const float accelY = ay * ACCEL_SCALE;
    const float accelZ = az * ACCEL_SCALE;

    const float accelRoll = atan2(accelY, accelZ) * RAD_TO_DEG;
    const float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;

    const float alpha = 0.98f;
    g_roll.angle = alpha * g_roll.angle + (1.0f - alpha) * accelRoll;
    g_pitch.angle = alpha * g_pitch.angle + (1.0f - alpha) * accelPitch;

    const float accelZWorld = (accelZ * cos(g_pitch.angle * DEG_TO_RAD) * cos(g_roll.angle * DEG_TO_RAD)
                               - accelY * sin(g_roll.angle * DEG_TO_RAD)
                               + accelX * sin(g_pitch.angle * DEG_TO_RAD));

    const float worldAccZ = (accelZWorld - 1.0f) * 981.0f; // Convert g to cm/s^2
    g_verticalVelocity += worldAccZ * dt;
    g_verticalVelocity *= 0.99f; // Simple damping
    g_altitudeCm += g_verticalVelocity * dt;
    if (g_altitudeCm < 0.0f)
    {
        g_altitudeCm = 0.0f;
    }
}

// ---------------------------------------------------------------------------
// Control
// ---------------------------------------------------------------------------

float mapControlToAngle(int16_t input)
{
    const float normalized = static_cast<float>(input) / 500.0f;
    return constrain(normalized * 30.0f, -35.0f, 35.0f);
}

float mapControlToYawRate(int16_t input)
{
    const float normalized = static_cast<float>(input) / 500.0f;
    return constrain(normalized * 120.0f, -180.0f, 180.0f);
}

void updateMotors(float dt)
{
    uint16_t throttle = ESC_MIN_US;
    float rollTarget = 0.0f;
    float pitchTarget = 0.0f;
    float yawTargetRate = 0.0f;

    if (g_linkEstablished)
    {
        throttle = constrainMicroseconds(g_lastControl.throttle);
        rollTarget = mapControlToAngle(applyDeadband(g_lastControl.roll, 6));
        pitchTarget = mapControlToAngle(applyDeadband(g_lastControl.pitch, 6));
        yawTargetRate = mapControlToYawRate(applyDeadband(g_lastControl.yaw, 6));
    }

    if (!g_motorsArmed || g_state == SystemState::FAILSAFE)
    {
        writeAllMotors(ESC_MIN_US);
        return;
    }

    if (!g_motorsSpinning)
    {
        throttle = ESC_MIN_US;
    }
    else if (g_state == SystemState::WAIT_IDLE_SPIN)
    {
        throttle = ESC_IDLE_SPIN_US;
    }

    if (g_motorsSpinning && throttle < ESC_IDLE_SPIN_US)
    {
        throttle = ESC_IDLE_SPIN_US;
    }

    const float rollOutput = g_pidRoll.update(rollTarget, g_roll.angle, g_roll.rate, dt);
    const float pitchOutput = g_pidPitch.update(pitchTarget, g_pitch.angle, g_pitch.rate, dt);
    const float yawOutput = g_pidYaw.update(yawTargetRate, g_yaw.rate, g_yaw.rate, dt);

    int32_t motorPower[4];
    motorPower[0] = throttle + pitchOutput - rollOutput + yawOutput; // Front left
    motorPower[1] = throttle + pitchOutput + rollOutput - yawOutput; // Front right
    motorPower[2] = throttle - pitchOutput - rollOutput - yawOutput; // Rear left
    motorPower[3] = throttle - pitchOutput + rollOutput + yawOutput; // Rear right

    for (uint8_t i = 0; i < 4; ++i)
    {
        motorPower[i] = constrain(motorPower[i], g_calibration.escMin, g_calibration.escMax);
        motors[i].writeMicroseconds(static_cast<uint16_t>(motorPower[i]));
    }
}

// ---------------------------------------------------------------------------
// Setup & loop
// ---------------------------------------------------------------------------

void initRadio()
{
    if (!radio.begin())
    {
        beepSequence({400, 400, 400}, 150, 80);
        while (true)
        {
            blinkStatus(1, 200, 200);
        }
    }

    radio.setChannel(RADIO_CHANNEL);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_1MBPS);
    radio.setAutoAck(true);
    radio.setRetries(3, 5);
    radio.enableAckPayload();
    radio.openReadingPipe(1, RADIO_PIPE_ADDRESS);
    radio.openWritingPipe(RADIO_ACK_PIPE_ADDRESS);
    radio.startListening();
}

void initMotors()
{
    motors[0].attach(PIN_MOTOR_FL, ESC_MIN_US, ESC_MAX_US);
    motors[1].attach(PIN_MOTOR_FR, ESC_MIN_US, ESC_MAX_US);
    motors[2].attach(PIN_MOTOR_RL, ESC_MIN_US, ESC_MAX_US);
    motors[3].attach(PIN_MOTOR_RR, ESC_MIN_US, ESC_MAX_US);
    writeAllMotors(ESC_MIN_US);
}

void setup()
{
    pinMode(PIN_STATUS_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    Serial.begin(115200);
    blinkStatus(2);

    Wire.begin();
    Wire.setClock(400000);

    mpu.initialize();
    if (!mpu.testConnection())
    {
        beepSequence({300, 700, 300, 700}, 180, 60);
        while (true)
        {
            blinkStatus(2, 200, 200);
        }
    }

    loadCalibration();

    if (g_calibration.signature == EEPROM_SIGNATURE)
    {
        mpu.setXAccelOffset(g_calibration.accelOffset[0]);
        mpu.setYAccelOffset(g_calibration.accelOffset[1]);
        mpu.setZAccelOffset(g_calibration.accelOffset[2]);
        mpu.setXGyroOffset(g_calibration.gyroOffset[0]);
        mpu.setYGyroOffset(g_calibration.gyroOffset[1]);
        mpu.setZGyroOffset(g_calibration.gyroOffset[2]);
    }

    initMotors();
    initRadio();

    g_lastLoopMicros = micros();
    g_lastControlMicros = micros();

    beepSequence({1200, 1600, 2000}, 90, 40);
}

void loop()
{
    const uint32_t nowMicros = micros();
    float dt = (nowMicros - g_lastLoopMicros) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.02f)
    {
        dt = LOOP_DT;
    }
    g_lastLoopMicros = nowMicros;

    pollRadio();
    updateImu(dt);
    updateMotors(dt);

    const uint32_t nowMillis = millis();
    if (nowMillis - g_lastTelemetryMillis >= 40)
    {
        g_lastTelemetryMillis = nowMillis;
        sendTelemetry();
    }
}

