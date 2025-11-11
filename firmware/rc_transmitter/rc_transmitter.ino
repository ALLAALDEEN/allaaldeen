#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>
#include <Adafruit_PCD8544.h>

#include "../shared/comm_protocol.h"

using namespace drone;

// ---------------------------------------------------------------------------
// Pin assignments (Arduino Nano)
// ---------------------------------------------------------------------------

constexpr uint8_t PIN_BUTTON_CALIBRATE = 2;
constexpr uint8_t PIN_BUTTON_IDLE_SPIN = 3;
constexpr uint8_t PIN_STATUS_LED = 4;

constexpr uint8_t LCD_RST = 5;
constexpr uint8_t LCD_DC = 6;
constexpr uint8_t LCD_CS = 7;

constexpr uint8_t PIN_TOGGLE_ARM = 8;   // Toggle switch (LOW = kill, HIGH = safe)

constexpr uint8_t RADIO_PIN_CE = 9;
constexpr uint8_t RADIO_PIN_CSN = 10;

// Analog channels
constexpr uint8_t PIN_JOYSTICK_THROTTLE = A0;
constexpr uint8_t PIN_JOYSTICK_YAW = A1;
constexpr uint8_t PIN_JOYSTICK_PITCH = A2;
constexpr uint8_t PIN_JOYSTICK_ROLL = A3;
constexpr uint8_t PIN_BATTERY_SENSE = A6;

constexpr uint8_t EEPROM_SIGNATURE = 0xAA;

// ---------------------------------------------------------------------------
// UI & timing configuration
// ---------------------------------------------------------------------------

constexpr uint32_t CONTROL_FRAME_INTERVAL_US = 10000; // 100 Hz
constexpr uint32_t DISPLAY_UPDATE_INTERVAL_MS = 120;
constexpr uint32_t TELEMETRY_TIMEOUT_MS = 400;
constexpr uint8_t ACTION_PULSE_FRAMES = 5;

// ---------------------------------------------------------------------------
// Calibration data
// ---------------------------------------------------------------------------

struct AxisCalibration
{
    uint16_t minRaw = 200;
    uint16_t centerRaw = 512;
    uint16_t maxRaw = 820;
};

struct RcCalibration
{
    AxisCalibration throttle;
    AxisCalibration yaw;
    AxisCalibration pitch;
    AxisCalibration roll;
    uint8_t signature = EEPROM_SIGNATURE;
};

RcCalibration g_calibration{};
bool g_calibrationValid = false;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

RF24 radio(RADIO_PIN_CE, RADIO_PIN_CSN);
Adafruit_PCD8544 display(LCD_DC, LCD_CS, LCD_RST);

ControlFrame g_control{};
TelemetryFrame g_lastTelemetry{};

bool g_linkOk = false;
bool g_newTelemetry = false;
uint32_t g_lastTelemetryMillis = 0;

uint32_t g_lastSendMicros = 0;
uint32_t g_lastDisplayMillis = 0;

uint8_t g_actionPulseMask = 0;
uint8_t g_actionPulseCountdown = 0;

bool g_ledActive = false;
uint32_t g_ledOffMillis = 0;

bool g_forceKillOverride = false;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void pulseStatusLed(uint16_t durationMs = 40)
{
    digitalWrite(PIN_STATUS_LED, HIGH);
    g_ledActive = true;
    g_ledOffMillis = millis() + durationMs;
}

void serviceStatusLed()
{
    if (g_ledActive && millis() >= g_ledOffMillis)
    {
        digitalWrite(PIN_STATUS_LED, LOW);
        g_ledActive = false;
    }
}

void loadCalibration()
{
    EEPROM.get(0, g_calibration);
    g_calibrationValid = (g_calibration.signature == EEPROM_SIGNATURE);
    if (!g_calibrationValid)
    {
        g_calibration = RcCalibration{};
    }
}

void saveCalibration()
{
    g_calibration.signature = EEPROM_SIGNATURE;
    EEPROM.put(0, g_calibration);
    g_calibrationValid = true;
}

uint16_t sampleAnalog(uint8_t pin, uint8_t samples = 8)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; ++i)
    {
        sum += analogRead(pin);
    }
    return sum / samples;
}

uint16_t mapThrottle(uint16_t raw)
{
    const AxisCalibration &cal = g_calibration.throttle;
    const uint16_t clamped = constrain(raw, cal.minRaw, cal.maxRaw);
    if (cal.maxRaw == cal.minRaw)
    {
        return 1000;
    }
    return map(clamped, cal.minRaw, cal.maxRaw, 1000, 2000);
}

int16_t mapCenteredAxis(const AxisCalibration &cal, uint16_t raw)
{
    const int32_t minRaw = cal.minRaw;
    const int32_t maxRaw = cal.maxRaw;
    const int32_t centerRaw = cal.centerRaw;
    int32_t output = 0;
    if (raw >= centerRaw)
    {
        const int32_t span = maxRaw - centerRaw;
        if (span <= 0)
        {
            return 0;
        }
        output = (static_cast<int32_t>(raw) - centerRaw) * 500 / span;
    }
    else
    {
        const int32_t span = centerRaw - minRaw;
        if (span <= 0)
        {
            return 0;
        }
        output = -((centerRaw - static_cast<int32_t>(raw)) * 500 / span);
    }
    const int16_t deadband = 6;
    if (abs(output) <= deadband)
    {
        return 0;
    }
    return constrain(output, -500, 500);
}

float analogToBatteryVoltage(uint16_t raw)
{
    constexpr float VREF = 5.0f;
    constexpr float DIVIDER_RATIO = 2.0f; // e.g., 1:1 divider for handheld pack
    return (static_cast<float>(raw) * VREF / 1023.0f) * DIVIDER_RATIO;
}

// ---------------------------------------------------------------------------
// Stick calibration
// ---------------------------------------------------------------------------

void drawCalibrationMessage(const char *line1, const char *line2)
{
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println(F("CAL MODE"));
    display.println(line1);
    display.println(line2);
    display.display();
}

void performStickCalibration()
{
    AxisCalibration throttle{};
    AxisCalibration yaw{};
    AxisCalibration pitch{};
    AxisCalibration roll{};

    g_forceKillOverride = true;

    throttle.minRaw = throttle.maxRaw = sampleAnalog(PIN_JOYSTICK_THROTTLE);
    yaw.minRaw = yaw.maxRaw = sampleAnalog(PIN_JOYSTICK_YAW);
    pitch.minRaw = pitch.maxRaw = sampleAnalog(PIN_JOYSTICK_PITCH);
    roll.minRaw = roll.maxRaw = sampleAnalog(PIN_JOYSTICK_ROLL);

    drawCalibrationMessage("Move all sticks", "Full travel!");
    const uint32_t travelStart = millis();
    while (millis() - travelStart < 5000UL)
    {
        throttle.minRaw = min(throttle.minRaw, analogRead(PIN_JOYSTICK_THROTTLE));
        throttle.maxRaw = max(throttle.maxRaw, analogRead(PIN_JOYSTICK_THROTTLE));
        yaw.minRaw = min(yaw.minRaw, analogRead(PIN_JOYSTICK_YAW));
        yaw.maxRaw = max(yaw.maxRaw, analogRead(PIN_JOYSTICK_YAW));
        pitch.minRaw = min(pitch.minRaw, analogRead(PIN_JOYSTICK_PITCH));
        pitch.maxRaw = max(pitch.maxRaw, analogRead(PIN_JOYSTICK_PITCH));
        roll.minRaw = min(roll.minRaw, analogRead(PIN_JOYSTICK_ROLL));
        roll.maxRaw = max(roll.maxRaw, analogRead(PIN_JOYSTICK_ROLL));
        delay(5);
        const uint32_t nowMicros = micros();
        if (nowMicros - g_lastSendMicros >= CONTROL_FRAME_INTERVAL_US)
        {
            g_lastSendMicros = nowMicros;
            sendControlFrame();
        }
    }

    drawCalibrationMessage("Release sticks", "Leave centered");
    const uint32_t centerStart = millis();
    uint32_t throttleAccum = 0;
    uint32_t yawAccum = 0;
    uint32_t pitchAccum = 0;
    uint32_t rollAccum = 0;
    uint16_t samples = 0;
    while (millis() - centerStart < 2500UL)
    {
        throttleAccum += analogRead(PIN_JOYSTICK_THROTTLE);
        yawAccum += analogRead(PIN_JOYSTICK_YAW);
        pitchAccum += analogRead(PIN_JOYSTICK_PITCH);
        rollAccum += analogRead(PIN_JOYSTICK_ROLL);
        ++samples;
        delay(5);
        const uint32_t nowMicros = micros();
        if (nowMicros - g_lastSendMicros >= CONTROL_FRAME_INTERVAL_US)
        {
            g_lastSendMicros = nowMicros;
            sendControlFrame();
        }
    }
    if (samples == 0)
    {
        samples = 1;
    }
    throttle.centerRaw = throttle.minRaw; // For throttle center is minimum position
    yaw.centerRaw = yawAccum / samples;
    pitch.centerRaw = pitchAccum / samples;
    roll.centerRaw = rollAccum / samples;

    throttle.minRaw = max<uint16_t>(0, throttle.minRaw - 10);
    throttle.maxRaw = min<uint16_t>(1023, throttle.maxRaw + 10);
    yaw.minRaw = max<uint16_t>(0, yaw.minRaw - 10);
    yaw.maxRaw = min<uint16_t>(1023, yaw.maxRaw + 10);
    pitch.minRaw = max<uint16_t>(0, pitch.minRaw - 10);
    pitch.maxRaw = min<uint16_t>(1023, pitch.maxRaw + 10);
    roll.minRaw = max<uint16_t>(0, roll.minRaw - 10);
    roll.maxRaw = min<uint16_t>(1023, roll.maxRaw + 10);

    g_calibration.throttle = throttle;
    g_calibration.yaw = yaw;
    g_calibration.pitch = pitch;
    g_calibration.roll = roll;
    saveCalibration();

    drawCalibrationMessage("Calibration", "Complete!");
    const uint32_t doneStart = millis();
    while (millis() - doneStart < 900UL)
    {
        const uint32_t nowMicros = micros();
        if (nowMicros - g_lastSendMicros >= CONTROL_FRAME_INTERVAL_US)
        {
            g_lastSendMicros = nowMicros;
            sendControlFrame();
        }
        delay(10);
    }

    g_forceKillOverride = false;
}

// ---------------------------------------------------------------------------
// Radio communication
// ---------------------------------------------------------------------------

bool radioWriteControlFrame()
{
    finalizeFrame(g_control);
    radio.stopListening();
    const bool ok = radio.write(&g_control, sizeof(g_control), false);
    radio.startListening();

    if (ok && radio.isAckPayloadAvailable())
    {
        TelemetryFrame telemetry{};
        radio.read(&telemetry, sizeof(telemetry));
        if (validateFrame(telemetry))
        {
            g_lastTelemetry = telemetry;
            g_lastTelemetryMillis = millis();
            g_linkOk = true;
            g_newTelemetry = true;
            pulseStatusLed();
        }
    }
    return ok;
}

void sendControlFrame()
{
    g_control.sequence++;
    g_control.reserved = 0;
    g_control.action_flags = g_actionPulseMask;
    uint8_t armMode = digitalRead(PIN_TOGGLE_ARM) == HIGH ? ARM_MODE_SAFE : ARM_MODE_KILL;
    if (g_forceKillOverride)
    {
        armMode = ARM_MODE_KILL;
    }
    g_control.arm_mode = armMode;

    const uint16_t rawThrottle = sampleAnalog(PIN_JOYSTICK_THROTTLE);
    const uint16_t rawYaw = sampleAnalog(PIN_JOYSTICK_YAW);
    const uint16_t rawPitch = sampleAnalog(PIN_JOYSTICK_PITCH);
    const uint16_t rawRoll = sampleAnalog(PIN_JOYSTICK_ROLL);

    if (g_calibrationValid)
    {
        g_control.throttle = mapThrottle(rawThrottle);
        g_control.yaw = mapCenteredAxis(g_calibration.yaw, rawYaw);
        g_control.pitch = mapCenteredAxis(g_calibration.pitch, rawPitch);
        g_control.roll = mapCenteredAxis(g_calibration.roll, rawRoll);
    }
    else
    {
        g_control.throttle = 1000;
        g_control.yaw = 0;
        g_control.pitch = 0;
        g_control.roll = 0;
    }

    if (g_forceKillOverride)
    {
        g_control.throttle = 1000;
        g_control.yaw = 0;
        g_control.pitch = 0;
        g_control.roll = 0;
    }

    radioWriteControlFrame();

    if (g_actionPulseCountdown > 0)
    {
        g_actionPulseCountdown--;
        if (g_actionPulseCountdown == 0)
        {
            g_actionPulseMask = 0;
        }
    }

    if (millis() - g_lastTelemetryMillis > TELEMETRY_TIMEOUT_MS)
    {
        g_linkOk = false;
    }
}

// ---------------------------------------------------------------------------
// Buttons & actions
// ---------------------------------------------------------------------------

bool readButton(uint8_t pin)
{
    return digitalRead(pin) == LOW;
}

void queueAction(uint8_t mask)
{
    g_actionPulseMask |= mask;
    g_actionPulseCountdown = ACTION_PULSE_FRAMES;
}

void handleButtons()
{
    static bool prevCalButton = false;
    static bool prevIdleButton = false;

    const bool calPressed = readButton(PIN_BUTTON_CALIBRATE);
    const bool idlePressed = readButton(PIN_BUTTON_IDLE_SPIN);

    if (calPressed && !prevCalButton)
    {
        performStickCalibration();
        queueAction(ACTION_REQUEST_CALIBRATION);
    }

    if (idlePressed && !prevIdleButton)
    {
        queueAction(ACTION_REQUEST_IDLE_SPIN);
    }

    prevCalButton = calPressed;
    prevIdleButton = idlePressed;
}

// ---------------------------------------------------------------------------
// Display rendering
// ---------------------------------------------------------------------------

const char *guidePhaseToText(uint8_t phase)
{
    switch (phase)
    {
    case GUIDE_RADIO_LINK:
        return "NRF link...";
    case GUIDE_SWITCH_KILL:
        return "Set toggle KILL";
    case GUIDE_CALIBRATE_PROMPT:
        return "Press BTN1 CAL";
    case GUIDE_CALIBRATING:
        return "Calibrating...";
    case GUIDE_ARM_PROMPT:
        return "Flip toggle ARM";
    case GUIDE_IDLE_SPIN_PROMPT:
        return "Press BTN2 Spin";
    case GUIDE_READY_TO_FLY:
        return "Ready to fly!";
    default:
        return "Awaiting...";
    }
}

void updateDisplay()
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    if (!g_linkOk)
    {
        display.println(F("LINK: --"));
        display.println(F("Searching FC"));
        if (!g_calibrationValid)
        {
            display.println(F("Run stick cal!"));
        }
        display.display();
        return;
    }

    const bool armed = (g_lastTelemetry.status_flags & STATUS_ARMED) != 0;
    const bool motors = (g_lastTelemetry.status_flags & STATUS_MOTORS_SPINNING) != 0;
    const bool calibrating = (g_lastTelemetry.status_flags & STATUS_CALIBRATING) != 0;

    display.print(F("LINK:OK "));
    display.print(armed ? F("ARM ") : F("SAFE"));
    display.print(motors ? F(" SPIN") : F(""));
    display.println();

    const char *guide = guidePhaseToText(g_lastTelemetry.guide_phase);
    display.println(guide);

    display.print(F("THR:"));
    display.print(g_control.throttle);
    display.print(F(" Y:"));
    display.println(g_control.yaw);

    display.print(F("P:"));
    display.print(g_control.pitch);
    display.print(F(" R:"));
    display.println(g_control.roll);

    display.print(F("Alt:"));
    display.print(g_lastTelemetry.altitude_cm);
    display.print(F("cm "));
    display.print(F("Bat:"));
    display.print(g_lastTelemetry.battery_mv / 1000.0f, 1);
    display.println(F("V"));
    const float rcBattery = analogToBatteryVoltage(analogRead(PIN_BATTERY_SENSE));
    display.print(F("RC:"));
    display.print(rcBattery, 2);
    display.println(F("V"));

    if (calibrating)
    {
        display.println(F("FC Busy..."));
    }

    display.display();
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

void initDisplay()
{
    display.begin();
    display.setContrast(55);
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("Quad RC Ready"));
    display.display();
}

void initRadio()
{
    if (!radio.begin())
    {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("NRF init fail!"));
        display.display();
        while (true)
        {
            digitalWrite(PIN_STATUS_LED, HIGH);
            delay(200);
            digitalWrite(PIN_STATUS_LED, LOW);
            delay(200);
        }
    }
    radio.setChannel(RADIO_CHANNEL);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_1MBPS);
    radio.setPayloadSize(sizeof(ControlFrame));
    radio.enableAckPayload();
    radio.setRetries(3, 5);
    radio.openWritingPipe(RADIO_PIPE_ADDRESS);
    radio.openReadingPipe(1, RADIO_ACK_PIPE_ADDRESS);
    radio.startListening();
}

// ---------------------------------------------------------------------------
// Arduino life-cycle
// ---------------------------------------------------------------------------

void setup()
{
    pinMode(PIN_BUTTON_CALIBRATE, INPUT_PULLUP);
    pinMode(PIN_BUTTON_IDLE_SPIN, INPUT_PULLUP);
    pinMode(PIN_TOGGLE_ARM, INPUT_PULLUP);
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    loadCalibration();
    initDisplay();
    initRadio();

    g_control.throttle = 1000;
    g_control.roll = 0;
    g_control.pitch = 0;
    g_control.yaw = 0;
    g_control.arm_mode = ARM_MODE_KILL;
    finalizeFrame(g_control);
}

void loop()
{
    serviceStatusLed();
    handleButtons();

    const uint32_t nowMicros = micros();
    if (nowMicros - g_lastSendMicros >= CONTROL_FRAME_INTERVAL_US)
    {
        g_lastSendMicros = nowMicros;
        sendControlFrame();
    }

    const uint32_t nowMillis = millis();
    if (nowMillis - g_lastDisplayMillis >= DISPLAY_UPDATE_INTERVAL_MS)
    {
        g_lastDisplayMillis = nowMillis;
        updateDisplay();
    }
}

