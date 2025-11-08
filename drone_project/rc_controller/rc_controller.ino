/**
 * RC Transmitter Firmware for Arduino Nano
 *
 * Hardware summary:
 *  - MCU: Arduino Nano
 *  - Radio: nRF24L01 (CE -> D9, CSN -> D10)
 *  - Joysticks: Left (Throttle/Yaw) -> V:A0, H:A1; Right (Pitch/Roll) -> V:A2, H:A3
 *  - Joystick push buttons -> A4, A5 (active LOW)
 *  - Buttons: D4, D5, D6, D7 (active LOW)
 *  - Toggle switches: SW1->D2, SW2->D3 (to GND = ON)
 *  - Potentiometers: A6, A7
 *
 * External library dependencies:
 *  - RF24 by TMRh20
 *
 * Author: GPT-5 Codex
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

// ------------------------------ Pins ---------------------------------------

constexpr uint8_t RADIO_CE_PIN = 9;
constexpr uint8_t RADIO_CSN_PIN = 10;
constexpr byte RADIO_ADDRESS[6] = "DRNFC";
constexpr uint8_t RADIO_CHANNEL = 100;
constexpr rf24_datarate_e RADIO_DATARATE = RF24_1MBPS;

// Joystick axes
constexpr uint8_t THROTTLE_PIN = A0;
constexpr uint8_t YAW_PIN = A1;
constexpr uint8_t PITCH_PIN = A2;
constexpr uint8_t ROLL_PIN = A3;
constexpr uint8_t JOYSTICK_L_BTN_PIN = A4;
constexpr uint8_t JOYSTICK_R_BTN_PIN = A5;

// Buttons (active LOW)
constexpr uint8_t BUTTON_1_PIN = 4;
constexpr uint8_t BUTTON_2_PIN = 5;
constexpr uint8_t BUTTON_3_PIN = 6;
constexpr uint8_t BUTTON_4_PIN = 7;

// Switches (active LOW)
constexpr uint8_t SWITCH_1_PIN = 2;
constexpr uint8_t SWITCH_2_PIN = 3;

// Potentiometers
constexpr uint8_t POT_1_PIN = A6;
constexpr uint8_t POT_2_PIN = A7;

// -------------------------- Radio structures -------------------------------

struct ControlPacket {
  uint32_t frameId;
  int16_t throttle;   // 0 -> 1000 (ESC microseconds offset)
  int16_t roll;       // -500 -> 500 (desired deg * 10)
  int16_t pitch;      // -500 -> 500
  int16_t yaw;        // -500 -> 500
  uint16_t buttons;
  uint8_t switches;
  uint16_t pots[2];
  uint8_t checksum;
};

struct TelemetryPacket {
  uint32_t frameId;
  float roll;
  float pitch;
  float yawRate;
  float altitudeMeters;
  uint16_t batteryMv;
  uint8_t status;
};

// ------------------------------ Globals ------------------------------------

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
ControlPacket txPacket{};
TelemetryPacket rxTelemetry{};
uint32_t frameCounter = 1;
uint32_t lastSendMillis = 0;

constexpr uint16_t RADIO_PERIOD_MS = 20;  // 50 Hz

// Calibration storage
struct AxisCalibration {
  int center;
  int min;
  int max;
};

AxisCalibration yawCal{512, 0, 1023};
AxisCalibration pitchCal{512, 0, 1023};
AxisCalibration rollCal{512, 0, 1023};
AxisCalibration throttleCal{0, 0, 1023};

constexpr uint16_t ANALOG_SAMPLES = 8;
constexpr int JOYSTICK_DEADBAND = 20;

// --------------------------- Utility functions -----------------------------

uint16_t readAnalogAveraged(uint8_t pin) {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < ANALOG_SAMPLES; ++i) {
    sum += analogRead(pin);
  }
  return sum / ANALOG_SAMPLES;
}

int16_t applyDeadband(int16_t value, int deadband) {
  if (abs(value) < deadband) {
    return 0;
  }
  return value;
}

int16_t mapAxisBipolar(uint16_t raw, const AxisCalibration &cal) {
  int32_t value = (int32_t)raw - cal.center;
  if (value > 0) {
    int32_t range = cal.max - cal.center;
    range = max(range, 1);
    value = (value * 500) / range;
  } else {
    int32_t range = cal.center - cal.min;
    range = max(range, 1);
    value = (value * 500) / range;
  }
  value = applyDeadband(value, JOYSTICK_DEADBAND);
  return constrain(value, -500, 500);
}

int16_t mapThrottleAxis(uint16_t raw, const AxisCalibration &cal) {
  int32_t value = (int32_t)raw - cal.min;
  int32_t span = cal.max - cal.min;
  span = max(span, 1);
  value = (value * 1000) / span;
  return constrain(value, 0, 1000);
}

uint8_t computeChecksum(const ControlPacket &packet) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(ControlPacket) - 1; ++i) {
    sum ^= bytes[i];
  }
  return sum;
}

// ---------------------------- Calibration ----------------------------------

AxisCalibration captureAxisCalibration(uint8_t pin, bool isThrottle) {
  AxisCalibration cal{};
  cal.min = 1023;
  cal.max = 0;
  uint32_t sum = 0;
  const uint16_t samples = 400;

  for (uint16_t i = 0; i < samples; ++i) {
    uint16_t value = analogRead(pin);
    cal.min = min(cal.min, (int)value);
    cal.max = max(cal.max, (int)value);
    sum += value;
    delay(2);
  }

  cal.center = isThrottle ? cal.min : (sum / samples);
  return cal;
}

void calibrateSticks() {
  Serial.println(F("[CAL] Center all sticks then hold BUTTON 4 to set extremes."));
  delay(1500);

  yawCal = captureAxisCalibration(YAW_PIN, false);
  pitchCal = captureAxisCalibration(PITCH_PIN, false);
  rollCal = captureAxisCalibration(ROLL_PIN, false);
  throttleCal = captureAxisCalibration(THROTTLE_PIN, true);

  Serial.print(F("[CAL] Yaw center: ")); Serial.print(yawCal.center);
  Serial.print(F(" min: ")); Serial.print(yawCal.min);
  Serial.print(F(" max: ")); Serial.println(yawCal.max);

  Serial.print(F("[CAL] Pitch center: ")); Serial.print(pitchCal.center);
  Serial.print(F(" min: ")); Serial.print(pitchCal.min);
  Serial.print(F(" max: ")); Serial.println(pitchCal.max);

  Serial.print(F("[CAL] Roll center: ")); Serial.print(rollCal.center);
  Serial.print(F(" min: ")); Serial.print(rollCal.min);
  Serial.print(F(" max: ")); Serial.println(rollCal.max);

  Serial.print(F("[CAL] Throttle min: ")); Serial.print(throttleCal.min);
  Serial.print(F(" max: ")); Serial.println(throttleCal.max);
}

// --------------------------- Input gathering --------------------------------

uint16_t readButtons() {
  uint16_t mask = 0;
  mask |= (digitalRead(BUTTON_1_PIN) == LOW) ? (1 << 0) : 0;
  mask |= (digitalRead(BUTTON_2_PIN) == LOW) ? (1 << 1) : 0;
  mask |= (digitalRead(BUTTON_3_PIN) == LOW) ? (1 << 2) : 0;
  mask |= (digitalRead(BUTTON_4_PIN) == LOW) ? (1 << 3) : 0;
  mask |= (digitalRead(JOYSTICK_L_BTN_PIN) == LOW) ? (1 << 4) : 0;
  mask |= (digitalRead(JOYSTICK_R_BTN_PIN) == LOW) ? (1 << 5) : 0;
  return mask;
}

uint8_t readSwitches() {
  uint8_t mask = 0;
  mask |= (digitalRead(SWITCH_1_PIN) == LOW) ? (1 << 0) : 0;
  mask |= (digitalRead(SWITCH_2_PIN) == LOW) ? (1 << 1) : 0;
  return mask;
}

void gatherInputs() {
  uint16_t rawThrottle = readAnalogAveraged(THROTTLE_PIN);
  uint16_t rawYaw = readAnalogAveraged(YAW_PIN);
  uint16_t rawPitch = readAnalogAveraged(PITCH_PIN);
  uint16_t rawRoll = readAnalogAveraged(ROLL_PIN);

  txPacket.throttle = mapThrottleAxis(rawThrottle, throttleCal);
  txPacket.yaw = mapAxisBipolar(rawYaw, yawCal);
  txPacket.pitch = mapAxisBipolar(rawPitch, pitchCal);
  txPacket.roll = mapAxisBipolar(rawRoll, rollCal);

  txPacket.buttons = readButtons();
  txPacket.switches = readSwitches();
  txPacket.pots[0] = analogRead(POT_1_PIN);
  txPacket.pots[1] = analogRead(POT_2_PIN);
}

// --------------------------- Radio interface --------------------------------

void setupRadio() {
  if (!radio.begin()) {
    Serial.println(F("[ERR] Radio not detected"));
  }
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RADIO_DATARATE);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openWritingPipe(RADIO_ADDRESS);
  radio.stopListening();
}

void sendControlFrame() {
  txPacket.frameId = frameCounter++;
  txPacket.checksum = computeChecksum(txPacket);

  bool ok = radio.write(&txPacket, sizeof(txPacket));
  if (!ok) {
    Serial.println(F("[WARN] Radio send failed"));
  }

  if (radio.isAckPayloadAvailable()) {
    radio.read(&rxTelemetry, sizeof(rxTelemetry));
    Serial.print(F("[TELEM] Frame ")); Serial.print(rxTelemetry.frameId);
    Serial.print(F(" Roll: ")); Serial.print(rxTelemetry.roll, 1);
    Serial.print(F(" Pitch: ")); Serial.print(rxTelemetry.pitch, 1);
    Serial.print(F(" Alt: ")); Serial.print(rxTelemetry.altitudeMeters, 1);
    Serial.print(F(" Batt: ")); Serial.print(rxTelemetry.batteryMv);
    Serial.print(F(" Status: 0x")); Serial.println(rxTelemetry.status, HEX);
  }
}

// ------------------------------ Setup --------------------------------------

void setupPins() {
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);
  pinMode(BUTTON_4_PIN, INPUT_PULLUP);
  pinMode(SWITCH_1_PIN, INPUT_PULLUP);
  pinMode(SWITCH_2_PIN, INPUT_PULLUP);
  pinMode(JOYSTICK_L_BTN_PIN, INPUT_PULLUP);
  pinMode(JOYSTICK_R_BTN_PIN, INPUT_PULLUP);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("[BOOT] RC transmitter starting"));

  setupPins();
  setupRadio();

  if (digitalRead(BUTTON_4_PIN) == LOW) {
    calibrateSticks();
  }
}

// ------------------------------- Loop --------------------------------------

void loop() {
  const uint32_t now = millis();
  if (now - lastSendMillis < RADIO_PERIOD_MS) {
    return;
  }
  lastSendMillis = now;

  gatherInputs();
  sendControlFrame();
}
