#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include "../shared/RadioPacket.h"

using namespace DroneLink;

// ---------------------------------------------------------------------------
// Hardware mapping (RC transmitter side)
// ---------------------------------------------------------------------------
constexpr uint8_t CE_PIN = 9;
constexpr uint8_t CSN_PIN = 10;

constexpr uint8_t JOY_LEFT_VERT = A0;  // Throttle
constexpr uint8_t JOY_LEFT_HORI = A1;  // Yaw
constexpr uint8_t JOY_LEFT_SEL = A4;   // Press-in switch

constexpr uint8_t JOY_RIGHT_VERT = A2;  // Pitch
constexpr uint8_t JOY_RIGHT_HORI = A3;  // Roll
constexpr uint8_t JOY_RIGHT_SEL = A5;   // Press-in switch

constexpr uint8_t BUTTON_1_PIN = 4;
constexpr uint8_t BUTTON_2_PIN = 5;
constexpr uint8_t BUTTON_3_PIN = 6;
constexpr uint8_t BUTTON_4_PIN = 7;

constexpr uint8_t SWITCH_1_PIN = 2;
constexpr uint8_t SWITCH_2_PIN = 3;

constexpr uint8_t POT_1_PIN = A6;
constexpr uint8_t POT_2_PIN = A7;

// Optional: solder a small status LED to D8.
constexpr uint8_t STATUS_LED_PIN = 8;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
constexpr uint16_t RADIO_RETRY_DELAY_MICROS = 1500;
constexpr uint8_t RADIO_RETRY_COUNT = 5;
constexpr unsigned long RADIO_PERIOD_MS = 15;  // ~66 Hz update rate

constexpr int16_t AXIS_DEADBAND = 15;
constexpr float AXIS_SCALE = 500.0f / 480.0f;  // 480 ~= half stick throw in raw counts

struct AxisCalibration {
  int16_t center = 512;
};

struct ThrottleCalibration {
  int16_t minVal = 0;
  int16_t maxVal = 1023;
};

RF24 radio(CE_PIN, CSN_PIN);
RadioPacket packet{};

AxisCalibration yawCal{};
AxisCalibration pitchCal{};
AxisCalibration rollCal{};
ThrottleCalibration throttleCal{};

uint16_t frameCounter = 0;
unsigned long lastSendMs = 0;

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------
int16_t applyDeadband(int16_t value, int16_t deadband) {
  if (abs(value) <= deadband) {
    return 0;
  }
  if (value > 0) {
    return value - deadband;
  }
  return value + deadband;
}

int16_t readAxis(uint8_t pin, const AxisCalibration &cal, bool invert = false) {
  int16_t raw = analogRead(pin);
  int16_t offset = raw - cal.center;
  int16_t value = static_cast<int16_t>(offset * AXIS_SCALE);
  value = applyDeadband(value, AXIS_DEADBAND);
  value = constrain(value, -500, 500);
  return invert ? -value : value;
}

uint16_t readThrottle(uint8_t pin, const ThrottleCalibration &cal) {
  int16_t raw = analogRead(pin);
  raw = constrain(raw, cal.minVal, cal.maxVal);
  int32_t mapped = map(raw, cal.minVal, cal.maxVal, 0, 1000);
  if (mapped < AXIS_DEADBAND) {
    mapped = 0;  // keep motors off when stick fully down
  }
  return static_cast<uint16_t>(constrain(mapped, 0, 1000));
}

uint16_t readKnob(uint8_t pin) {
  int16_t raw = analogRead(pin);
  return static_cast<uint16_t>(constrain(raw, 0, 1023));
}

uint16_t readSwitches() {
  uint16_t mask = 0;
  if (digitalRead(SWITCH_1_PIN) == LOW) {
    mask |= (1 << 0);
  }
  if (digitalRead(SWITCH_2_PIN) == LOW) {
    mask |= (1 << 1);
  }
  if (digitalRead(BUTTON_1_PIN) == LOW) {
    mask |= (1 << 2);
  }
  if (digitalRead(BUTTON_2_PIN) == LOW) {
    mask |= (1 << 3);
  }
  if (digitalRead(BUTTON_3_PIN) == LOW) {
    mask |= (1 << 4);
  }
  if (digitalRead(BUTTON_4_PIN) == LOW) {
    mask |= (1 << 5);
  }
  if (digitalRead(JOY_LEFT_SEL) == LOW) {
    mask |= (1 << 6);
  }
  if (digitalRead(JOY_RIGHT_SEL) == LOW) {
    mask |= (1 << 7);
  }
  return mask;
}

void calibrateJoysticks() {
  constexpr size_t samples = 400;
  long yawSum = 0;
  long rollSum = 0;
  long pitchSum = 0;
  long throttleMin = 1023;
  long throttleMax = 0;

  for (size_t i = 0; i < samples; ++i) {
    yawSum += analogRead(JOY_LEFT_HORI);
    rollSum += analogRead(JOY_RIGHT_HORI);
    pitchSum += analogRead(JOY_RIGHT_VERT);

    int16_t throttleRaw = analogRead(JOY_LEFT_VERT);
    if (throttleRaw < throttleMin) {
      throttleMin = throttleRaw;
    }
    if (throttleRaw > throttleMax) {
      throttleMax = throttleRaw;
    }
    delay(2);
  }

  yawCal.center = yawSum / samples;
  rollCal.center = rollSum / samples;
  pitchCal.center = pitchSum / samples;

  // Allow a bit of headroom so the throttle can hit 0 and 1000 reliably.
  throttleCal.minVal = max<int16_t>(0, throttleMin - 10);
  throttleCal.maxVal = min<int16_t>(1023, throttleMax + 10);
}

void setupInputs() {
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);
  pinMode(BUTTON_4_PIN, INPUT_PULLUP);

  pinMode(SWITCH_1_PIN, INPUT_PULLUP);
  pinMode(SWITCH_2_PIN, INPUT_PULLUP);

  pinMode(JOY_LEFT_SEL, INPUT_PULLUP);
  pinMode(JOY_RIGHT_SEL, INPUT_PULLUP);

  pinMode(STATUS_LED_PIN, OUTPUT);
}

void setupRadio() {
  if (!radio.begin()) {
    // Stay in an error blink loop if the radio is not found.
    while (true) {
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(100);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(100);
    }
  }

  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76);  // Out of WiFi band
  radio.setRetries(RADIO_RETRY_DELAY_MICROS / 250, RADIO_RETRY_COUNT);
  radio.openWritingPipe(kRadioAddress);
  radio.stopListening();
}

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  setupInputs();
  calibrateJoysticks();
  setupRadio();

#ifdef SERIAL_PORT_MONITOR
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println(F("RC Controller ready"));
#endif
}

void sendPacket() {
  packet.throttle = readThrottle(JOY_LEFT_VERT, throttleCal);
  packet.yaw = readAxis(JOY_LEFT_HORI, yawCal);
  packet.pitch = readAxis(JOY_RIGHT_VERT, pitchCal, true);  // Up stick -> positive pitch
  packet.roll = readAxis(JOY_RIGHT_HORI, rollCal);
  packet.switches = readSwitches();
  packet.knobs[0] = readKnob(POT_1_PIN);
  packet.knobs[1] = readKnob(POT_2_PIN);
  packet.batteryMv = 0;  // Populate if you wire a voltage divider to A6/A7 instead of the potentiometers.
  packet.frameId = frameCounter++;
  finalizePacket(packet);

  bool ok = radio.write(&packet, sizeof(packet), false);
  digitalWrite(STATUS_LED_PIN, ok ? HIGH : LOW);

#ifdef SERIAL_PORT_MONITOR
  Serial.print(F("TX -> ")); Serial.print(packet.throttle);
  Serial.print(F(" | P:")); Serial.print(packet.pitch);
  Serial.print(F(" R:")); Serial.print(packet.roll);
  Serial.print(F(" Y:")); Serial.print(packet.yaw);
  Serial.print(F(" Sw:")); Serial.println(packet.switches, BIN);
#endif
}

void loop() {
  unsigned long now = millis();
  if (now - lastSendMs >= RADIO_PERIOD_MS) {
    sendPacket();
    lastSendMs = now;
  }
}

