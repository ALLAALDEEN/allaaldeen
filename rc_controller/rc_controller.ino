/**
 * DIY RC Transmitter for Quadcopter
 *
 * Hardware:
 *  - Arduino Nano
 *  - nRF24L01+ (CE=D9, CSN=D10)
 *  - Dual analog joysticks
 *  - 4 momentary buttons (D4-D7)
 *  - 2 toggle switches (SW1=D2, SW2=D3)
 *  - Potentiometers (A6, A7)
 *
 * Author: GPT-5 Codex
 */

#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>

#include "../shared/rc_protocol.h"

// ---------------------- Pin Assignments ----------------------
static constexpr uint8_t PIN_NRF_CE = 9;
static constexpr uint8_t PIN_NRF_CSN = 10;

static constexpr uint8_t PIN_LEFT_Y = A0;  // Throttle
static constexpr uint8_t PIN_LEFT_X = A1;  // Yaw
static constexpr uint8_t PIN_RIGHT_Y = A2; // Pitch
static constexpr uint8_t PIN_RIGHT_X = A3; // Roll

static constexpr uint8_t PIN_LEFT_BTN = A4;  // Joystick press
static constexpr uint8_t PIN_RIGHT_BTN = A5; // Joystick press

static constexpr uint8_t PIN_BUTTON_1 = 4;
static constexpr uint8_t PIN_BUTTON_2 = 5;
static constexpr uint8_t PIN_BUTTON_3 = 6;
static constexpr uint8_t PIN_BUTTON_4 = 7;

static constexpr uint8_t PIN_SWITCH_1 = 2;
static constexpr uint8_t PIN_SWITCH_2 = 3;

static constexpr uint8_t PIN_POT_1 = A6;
static constexpr uint8_t PIN_POT_2 = A7;

// ---------------------- Radio Configuration ----------------------
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const uint8_t kControllerAddress[6] = {'R', 'C', 'L', 'N', 'K'};
const uint8_t kTelemetryAddress[6] = {'D', 'R', 'O', 'N', 'E'};

// ---------------------- Control Constants ----------------------
static constexpr uint16_t BUTTON_ARM = 0x0001;
static constexpr uint16_t BUTTON_DISARM = 0x0002;
static constexpr uint16_t BUTTON_BEEP = 0x0004;
static constexpr uint16_t BUTTON_1_BIT = 0x0008;
static constexpr uint16_t BUTTON_2_BIT = 0x0010;
static constexpr uint16_t BUTTON_3_BIT = 0x0020;
static constexpr uint16_t BUTTON_4_BIT = 0x0040;
static constexpr uint16_t BUTTON_LEFT_PRESS = 0x0080;
static constexpr uint16_t BUTTON_RIGHT_PRESS = 0x0100;

static constexpr uint16_t SWITCH_MODE = 0x0001;
static constexpr uint16_t SWITCH_ALT_HOLD = 0x0002;

static constexpr uint8_t EEPROM_MAGIC_ADDR = 0;
static constexpr uint8_t EEPROM_MAGIC_VALUE = 0x42;
static constexpr uint8_t EEPROM_CALIB_BASE = 4;

struct AxisCalibration {
  int16_t minValue = 0;
  int16_t centerValue = 512;
  int16_t maxValue = 1023;
};

AxisCalibration calThrottle;
AxisCalibration calYaw;
AxisCalibration calPitch;
AxisCalibration calRoll;

struct AxisFilter {
  uint8_t pin;
  bool invert;
  float filtered;
};

AxisFilter axisThrottle{PIN_LEFT_Y, true, 0};
AxisFilter axisYaw{PIN_LEFT_X, false, 0};
AxisFilter axisPitch{PIN_RIGHT_Y, true, 0};
AxisFilter axisRoll{PIN_RIGHT_X, false, 0};

static constexpr float kFilterAlpha = 0.35f;
static constexpr uint32_t kPacketIntervalMs = 10;

rc::ControlPacket outboundPacket;
rc::TelemetryPacket inboundTelemetry;

void loadCalibration() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VALUE) {
    return;
  }
  EEPROM.get(EEPROM_CALIB_BASE + 0, calThrottle);
  EEPROM.get(EEPROM_CALIB_BASE + sizeof(AxisCalibration), calYaw);
  EEPROM.get(EEPROM_CALIB_BASE + 2 * sizeof(AxisCalibration), calPitch);
  EEPROM.get(EEPROM_CALIB_BASE + 3 * sizeof(AxisCalibration), calRoll);
}

void saveCalibration() {
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  EEPROM.put(EEPROM_CALIB_BASE + 0, calThrottle);
  EEPROM.put(EEPROM_CALIB_BASE + sizeof(AxisCalibration), calYaw);
  EEPROM.put(EEPROM_CALIB_BASE + 2 * sizeof(AxisCalibration), calPitch);
  EEPROM.put(EEPROM_CALIB_BASE + 3 * sizeof(AxisCalibration), calRoll);
}

void calibrateAxis(AxisCalibration& calibration, AxisFilter& filter, const char* name) {
  const uint16_t samples = 400;
  int32_t sum = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    int16_t sample = analogRead(filter.pin);
    if (filter.invert) {
      sample = 1023 - sample;
    }
    sum += sample;
    delay(2);
  }
  calibration.centerValue = sum / samples;

  calibration.minValue = 1023;
  calibration.maxValue = 0;
  uint32_t start = millis();
  Serial.print(F("Move "));
  Serial.print(name);
  Serial.println(F(" to extremes..."));
  while (millis() - start < 5000) {
    int16_t value = analogRead(filter.pin);
    if (filter.invert) {
      value = 1023 - value;
    }
    if (value < calibration.minValue) calibration.minValue = value;
    if (value > calibration.maxValue) calibration.maxValue = value;
  }
  Serial.print(name);
  Serial.print(F(" calibration -> min: "));
  Serial.print(calibration.minValue);
  Serial.print(F(" center: "));
  Serial.print(calibration.centerValue);
  Serial.print(F(" max: "));
  Serial.println(calibration.maxValue);
}

void performCalibration() {
  Serial.println(F("=== RC Calibration ==="));
  Serial.println(F("Keep sticks centered when prompted."));
  delay(500);

  calibrateAxis(calThrottle, axisThrottle, "Throttle");
  calibrateAxis(calYaw, axisYaw, "Yaw");
  calibrateAxis(calPitch, axisPitch, "Pitch");
  calibrateAxis(calRoll, axisRoll, "Roll");

  saveCalibration();
  Serial.println(F("Calibration stored to EEPROM."));
}

float readFiltered(AxisFilter& axis) {
  int16_t raw = analogRead(axis.pin);
  if (axis.invert) {
    raw = 1023 - raw;
  }
  axis.filtered = axis.filtered + kFilterAlpha * ((float)raw - axis.filtered);
  return axis.filtered;
}

int16_t applyCalibration(float value, const AxisCalibration& calibration, int16_t outMin, int16_t outMax) {
  if (value < calibration.minValue) value = calibration.minValue;
  if (value > calibration.maxValue) value = calibration.maxValue;

  int16_t inMin = calibration.minValue;
  int16_t inMax = calibration.maxValue;

  if (inMax == inMin) {
    return (outMin + outMax) / 2;
  }
  float ratio = (value - inMin) / (float)(inMax - inMin);
  if (ratio < 0.0f) ratio = 0.0f;
  if (ratio > 1.0f) ratio = 1.0f;
  float mapped = outMin + ratio * (outMax - outMin);
  return static_cast<int16_t>(mapped);
}

int16_t readStick(AxisFilter& axis, AxisCalibration& calibration, int16_t outMin, int16_t outMax) {
  float filtered = readFiltered(axis);
  int16_t mapped = applyCalibration(filtered, calibration, outMin, outMax);
  return constrain(mapped, outMin, outMax);
}

uint16_t readButtons() {
  uint16_t buttons = 0;
  if (digitalRead(PIN_BUTTON_1) == LOW) buttons |= BUTTON_1_BIT;
  if (digitalRead(PIN_BUTTON_2) == LOW) buttons |= BUTTON_2_BIT;
  if (digitalRead(PIN_BUTTON_3) == LOW) buttons |= BUTTON_3_BIT;
  if (digitalRead(PIN_BUTTON_4) == LOW) buttons |= BUTTON_4_BIT;
  if (digitalRead(PIN_LEFT_BTN) == LOW) buttons |= BUTTON_LEFT_PRESS;
  if (digitalRead(PIN_RIGHT_BTN) == LOW) buttons |= BUTTON_RIGHT_PRESS;

  // Map buttons to specific actions
  if (buttons & BUTTON_1_BIT) buttons |= BUTTON_ARM;
  if (buttons & BUTTON_2_BIT) buttons |= BUTTON_DISARM;
  if (buttons & BUTTON_3_BIT) buttons |= BUTTON_BEEP;
  return buttons;
}

uint16_t readSwitches() {
  uint16_t switches = 0;
  if (digitalRead(PIN_SWITCH_1) == LOW) switches |= SWITCH_MODE;
  if (digitalRead(PIN_SWITCH_2) == LOW) switches |= SWITCH_ALT_HOLD;
  return switches;
}

uint16_t readPot(uint8_t pin) {
  int raw = analogRead(pin);
  return constrain(map(raw, 0, 1023, 0, 1000), 0, 1000);
}

void setupRadio() {
  radio.begin();
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setRetries(5, 15);
  radio.setChannel(90);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openWritingPipe(kControllerAddress);
  radio.openReadingPipe(1, kTelemetryAddress);
  radio.startListening();
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("Starting RC transmitter..."));

  pinMode(PIN_BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_2, INPUT_PULLUP);
  pinMode(PIN_BUTTON_3, INPUT_PULLUP);
  pinMode(PIN_BUTTON_4, INPUT_PULLUP);
  pinMode(PIN_LEFT_BTN, INPUT_PULLUP);
  pinMode(PIN_RIGHT_BTN, INPUT_PULLUP);
  pinMode(PIN_SWITCH_1, INPUT_PULLUP);
  pinMode(PIN_SWITCH_2, INPUT_PULLUP);

  loadCalibration();

  axisThrottle.filtered = axisThrottle.invert ? (1023 - analogRead(axisThrottle.pin))
                                              : analogRead(axisThrottle.pin);
  axisYaw.filtered = axisYaw.invert ? (1023 - analogRead(axisYaw.pin))
                                    : analogRead(axisYaw.pin);
  axisPitch.filtered = axisPitch.invert ? (1023 - analogRead(axisPitch.pin))
                                        : analogRead(axisPitch.pin);
  axisRoll.filtered = axisRoll.invert ? (1023 - analogRead(axisRoll.pin))
                                      : analogRead(axisRoll.pin);

  setupRadio();

  Serial.println(F("Hold button 4 while powering on to force calibration."));
  delay(500);
  if (digitalRead(PIN_BUTTON_4) == LOW) {
    performCalibration();
  }

  Serial.println(F("RC transmitter ready."));
}

void sendPacket() {
  outboundPacket.magic = rc::kPacketMagic;
  outboundPacket.version = rc::kProtocolVersion;
  outboundPacket.throttle = readStick(axisThrottle, calThrottle, 1000, 2000);
  outboundPacket.yaw = readStick(axisYaw, calYaw, -500, 500);
  outboundPacket.pitch = readStick(axisPitch, calPitch, -500, 500);
  outboundPacket.roll = readStick(axisRoll, calRoll, -500, 500);

  outboundPacket.buttons = readButtons();
  outboundPacket.switches = readSwitches();
  outboundPacket.dial1 = readPot(PIN_POT_1);
  outboundPacket.dial2 = readPot(PIN_POT_2);

  outboundPacket.aux1 = outboundPacket.dial1;
  outboundPacket.aux2 = outboundPacket.dial2;

  rc::finalize(outboundPacket);

  radio.stopListening();
  bool ok = radio.write(&outboundPacket, sizeof(outboundPacket));
  radio.startListening();

  if (!ok) {
    Serial.println(F("Radio send failed"));
  }
}

void readTelemetry() {
  while (radio.available()) {
    radio.read(&inboundTelemetry, sizeof(inboundTelemetry));
    if (rc::validate(inboundTelemetry)) {
      Serial.print(F("Telemetry | Roll: "));
      Serial.print(inboundTelemetry.roll_deg / 10.0f);
      Serial.print(F(" Pitch: "));
      Serial.print(inboundTelemetry.pitch_deg / 10.0f);
      Serial.print(F(" Alt(cm): "));
      Serial.print(inboundTelemetry.altitude_cm);
      Serial.print(F(" Flags: 0x"));
      Serial.println(inboundTelemetry.status_flags, HEX);
    }
  }
}

void loop() {
  static uint32_t lastSendMs = 0;
  uint32_t now = millis();
  if (now - lastSendMs >= kPacketIntervalMs) {
    lastSendMs = now;
    sendPacket();
  }

  readTelemetry();
}
