/*
  RC Transmitter Firmware for Nano-based Quad Controller
  Hardware:
    - Arduino Nano
    - NRF24L01 (CE -> D9, CSN -> D10, SPI pins default)
    - Two XY joysticks (V/H axes on A0..A3, select buttons on A4/A5)
    - Four momentary buttons (D4..D7)
    - Two toggle switches (SW1 -> D2, SW2 -> D3, other pins -> GND)
    - Two potentiometers (A6, A7)

  Libraries required (install via Library Manager):
    - RF24 by TMRh20
*/

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

// -------------------- Radio --------------------
constexpr uint8_t PIN_RF_CE = 9;
constexpr uint8_t PIN_RF_CSN = 10;

RF24 radio(PIN_RF_CE, PIN_RF_CSN);
const byte RADIO_ADDRESS_TX[6] = "CTRL1";  // to flight controller
const byte RADIO_ADDRESS_RX[6] = "STAT1";  // telemetry back from flight controller

// -------------------- Control Inputs --------------------
constexpr uint8_t PIN_JOYL_VERT = A0;  // Throttle
constexpr uint8_t PIN_JOYL_HORZ = A1;  // Yaw
constexpr uint8_t PIN_JOYR_VERT = A2;  // Pitch
constexpr uint8_t PIN_JOYR_HORZ = A3;  // Roll
constexpr uint8_t PIN_JOYL_SEL = A4;
constexpr uint8_t PIN_JOYR_SEL = A5;

constexpr uint8_t PIN_BUTTON_1 = 4;
constexpr uint8_t PIN_BUTTON_2 = 5;
constexpr uint8_t PIN_BUTTON_3 = 6;
constexpr uint8_t PIN_BUTTON_4 = 7;

constexpr uint8_t PIN_SWITCH_1 = 2;
constexpr uint8_t PIN_SWITCH_2 = 3;

constexpr uint8_t PIN_POT_1 = A6;
constexpr uint8_t PIN_POT_2 = A7;

constexpr uint16_t ESC_MIN_US = 1000;
constexpr uint16_t ESC_MAX_US = 2000;

// -------------------- Frame Definitions --------------------
struct __attribute__((packed)) ControlFrame {
  uint16_t throttle;  // microseconds (1000-2000)
  int16_t yaw;        // centi-deg/s (-15000..15000)
  int16_t pitch;      // centi-deg (-3000..3000)
  int16_t roll;       // centi-deg (-3000..3000)
  uint8_t switches;   // bit0: SW1, bit1: SW2
  uint8_t buttons;    // bits0-3: buttons 1-4, bit4: L-joy press, bit5: R-joy press
  uint16_t pot1;      // 0-1023
  uint16_t pot2;      // 0-1023
  uint16_t checksum;
};

struct __attribute__((packed)) TelemetryFrame {
  float altitudeMeters;
  float batteryVolts;
  float pitchDeg;
  float rollDeg;
  float yawDeg;
  uint16_t checksum;
};

ControlFrame controlFrame{};
TelemetryFrame telemetryFrame{};

// -------------------- Timing --------------------
constexpr uint16_t LOOP_RATE_HZ = 100;
constexpr uint32_t LOOP_INTERVAL_MS = 1000 / LOOP_RATE_HZ;
unsigned long lastLoopMs = 0;

// -------------------- Utility --------------------
uint16_t computeChecksum(const uint8_t* data, size_t length) {
  uint16_t sum = 0;
  for (size_t i = 0; i < length; ++i) {
    sum += data[i];
  }
  return 0xFFFFu - sum;
}

int16_t mapCenteredAxis(int raw, int deadband, int16_t maxCentiUnits) {
  int16_t centered = raw - 512;
  if (abs(centered) < deadband) {
    centered = 0;
  }
  long scaled = static_cast<long>(centered) * maxCentiUnits;
  return constrain(scaled / 512, -maxCentiUnits, maxCentiUnits);
}

uint16_t mapThrottle(int raw) {
  return constrain(map(raw, 0, 1023, ESC_MIN_US, ESC_MAX_US), ESC_MIN_US, ESC_MAX_US);
}

uint8_t readSwitchBits() {
  uint8_t bits = 0;
  if (digitalRead(PIN_SWITCH_1) == LOW) bits |= 0x01;
  if (digitalRead(PIN_SWITCH_2) == LOW) bits |= 0x02;
  return bits;
}

uint8_t readButtonBits() {
  uint8_t bits = 0;
  if (digitalRead(PIN_BUTTON_1) == LOW) bits |= 0x01;
  if (digitalRead(PIN_BUTTON_2) == LOW) bits |= 0x02;
  if (digitalRead(PIN_BUTTON_3) == LOW) bits |= 0x04;
  if (digitalRead(PIN_BUTTON_4) == LOW) bits |= 0x08;
  if (digitalRead(PIN_JOYL_SEL) == LOW) bits |= 0x10;
  if (digitalRead(PIN_JOYR_SEL) == LOW) bits |= 0x20;
  return bits;
}

void readControls(ControlFrame& frame) {
  frame.throttle = mapThrottle(analogRead(PIN_JOYL_VERT));
  frame.yaw = mapCenteredAxis(analogRead(PIN_JOYL_HORZ), 12, 15000);
  frame.pitch = mapCenteredAxis(analogRead(PIN_JOYR_VERT), 12, 3000);
  frame.roll = mapCenteredAxis(analogRead(PIN_JOYR_HORZ), 12, 3000);
  frame.switches = readSwitchBits();
  frame.buttons = readButtonBits();
  frame.pot1 = analogRead(PIN_POT_1);
  frame.pot2 = analogRead(PIN_POT_2);
  frame.checksum = 0;
  frame.checksum = computeChecksum(reinterpret_cast<uint8_t*>(&frame), sizeof(ControlFrame));
}

bool validateTelemetry(TelemetryFrame& frame) {
  uint16_t receivedChecksum = frame.checksum;
  frame.checksum = 0;
  uint16_t expected = computeChecksum(reinterpret_cast<uint8_t*>(&frame), sizeof(TelemetryFrame));
  frame.checksum = receivedChecksum;
  return expected == receivedChecksum;
}

void setupPins() {
  pinMode(PIN_SWITCH_1, INPUT_PULLUP);
  pinMode(PIN_SWITCH_2, INPUT_PULLUP);

  pinMode(PIN_BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_2, INPUT_PULLUP);
  pinMode(PIN_BUTTON_3, INPUT_PULLUP);
  pinMode(PIN_BUTTON_4, INPUT_PULLUP);

  pinMode(PIN_JOYL_SEL, INPUT_PULLUP);
  pinMode(PIN_JOYR_SEL, INPUT_PULLUP);
}

void setupRadio() {
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(115);
  radio.setRetries(3, 15);
  radio.openWritingPipe(RADIO_ADDRESS_TX);
  radio.openReadingPipe(1, RADIO_ADDRESS_RX);
  radio.startListening();
}

void setup() {
  Serial.begin(115200);
  setupPins();
  setupRadio();
  lastLoopMs = millis();
}

void processTelemetry() {
  while (radio.available()) {
    radio.read(&telemetryFrame, sizeof(TelemetryFrame));
    if (validateTelemetry(telemetryFrame)) {
      Serial.print(F("Alt(m): "));
      Serial.print(telemetryFrame.altitudeMeters, 1);
      Serial.print(F("  Pitch: "));
      Serial.print(telemetryFrame.pitchDeg, 1);
      Serial.print(F("  Roll: "));
      Serial.print(telemetryFrame.rollDeg, 1);
      Serial.print(F("  Yaw: "));
      Serial.print(telemetryFrame.yawDeg, 1);
      Serial.print(F("  VBatt: "));
      Serial.println(telemetryFrame.batteryVolts, 2);
    }
  }
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoopMs >= LOOP_INTERVAL_MS) {
    lastLoopMs = now;

    readControls(controlFrame);

    radio.stopListening();
    bool ok = radio.write(&controlFrame, sizeof(ControlFrame));
    radio.startListening();

    if (!ok) {
      Serial.println(F("WARN: control frame not acknowledged"));
    }
  }

  processTelemetry();
}

