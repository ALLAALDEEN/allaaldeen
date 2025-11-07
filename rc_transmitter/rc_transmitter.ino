/*
 * NRF24-based RC transmitter for Arduino Nano
 *
 * Hardware:
 *  - Arduino Nano
 *  - NRF24L01 (CE=D9, CSN=D10)
 *  - Left joystick:  V=A0 (Throttle), H=A1 (Yaw), SEL=A4
 *  - Right joystick: V=A2 (Pitch),    H=A3 (Roll), SEL=A5
 *  - Push buttons: D4, D5, D6, D7
 *  - Toggles:      SW1=D2, SW2=D3 (connect other pins to GND)
 *  - Potentiometers: A6, A7
 *  - Optional status LED: D13
 *
 * Dependencies:
 *  - RF24 library from TMRh20
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <math.h>

// ---------------------------
// Pin assignments
// ---------------------------
const uint8_t PIN_NRF_CE = 9;
const uint8_t PIN_NRF_CSN = 10;

const uint8_t PIN_BTN_1 = 4;
const uint8_t PIN_BTN_2 = 5;
const uint8_t PIN_BTN_3 = 6;
const uint8_t PIN_BTN_4 = 7;

const uint8_t PIN_SW_1 = 2;
const uint8_t PIN_SW_2 = 3;

const uint8_t PIN_LED_STATUS = 13;

const uint8_t PIN_JOY_LEFT_V = A0; // throttle
const uint8_t PIN_JOY_LEFT_H = A1; // yaw
const uint8_t PIN_JOY_RIGHT_V = A2; // pitch
const uint8_t PIN_JOY_RIGHT_H = A3; // roll

const uint8_t PIN_JOY_LEFT_SEL = A4;
const uint8_t PIN_JOY_RIGHT_SEL = A5;

const uint8_t PIN_POT_1 = A6;
const uint8_t PIN_POT_2 = A7;

// ---------------------------
// RF24 configuration
// ---------------------------
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const byte RADIO_ADDRESS[6] = "DRON1";

// ---------------------------
// Control packet (matches flight controller)
// ---------------------------
struct ControlPacket {
  uint8_t magic[2];
  uint8_t version;
  uint8_t sequence;
  int16_t throttle;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  int16_t aux1;
  int16_t aux2;
  uint8_t buttons;
  uint8_t reserved;
  uint16_t checksum;
} __attribute__((packed));

enum ButtonBits : uint8_t {
  BTN_JOY_LEFT  = 0,
  BTN_JOY_RIGHT = 1,
  BTN_1         = 2,
  BTN_2         = 3,
  BTN_3         = 4,
  BTN_4         = 5,
  BTN_SW1       = 6,
  BTN_SW2       = 7
};

// ---------------------------
// Stick configuration
// ---------------------------
const int STICK_DEADBAND = 12;

const bool THROTTLE_INVERT = true;  // true so pushing stick up increases throttle
const bool YAW_INVERT = false;
const bool PITCH_INVERT = true;     // pushing forward (up) -> positive pitch
const bool ROLL_INVERT = false;

const bool POT1_INVERT = false;
const bool POT2_INVERT = false;

// ---------------------------
// Timing
// ---------------------------
const uint32_t TRANSMIT_PERIOD_US = 10000; // 100 Hz

// ---------------------------
// Globals
// ---------------------------
ControlPacket packet = { {'D', 'R'}, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t sequenceCounter = 0;
uint32_t lastTransmitUs = 0;
uint32_t lastBlinkMs = 0;
bool linkOk = false;

float rollCenterOffset = 0.0f;
float pitchCenterOffset = 0.0f;
float yawCenterOffset = 0.0f;

// ---------------------------
// Helpers
// ---------------------------
uint16_t computeChecksum(const ControlPacket &pkt) {
  const uint8_t *raw = reinterpret_cast<const uint8_t *>(&pkt);
  uint16_t sum = 0;
  for (size_t i = 0; i < sizeof(ControlPacket) - sizeof(pkt.checksum); ++i) {
    sum += raw[i];
  }
  return sum;
}

int16_t applyDeadband(int16_t value, int16_t deadband) {
  if (abs(value) < deadband) {
    return 0;
  }
  return value;
}

float readAnalogNormalized(uint8_t pin, bool invert) {
  int raw = analogRead(pin);
  float norm = static_cast<float>(raw) / 1023.0f;
  if (invert) {
    norm = 1.0f - norm;
  }
  return constrain(norm, 0.0f, 1.0f);
}

float readAnalogCentered(uint8_t pin, bool invert) {
  int raw = analogRead(pin);
  float center = 512.0f;
  if (pin == PIN_JOY_RIGHT_H) {
    center += rollCenterOffset;
  } else if (pin == PIN_JOY_RIGHT_V) {
    center += pitchCenterOffset;
  } else if (pin == PIN_JOY_LEFT_H) {
    center += yawCenterOffset;
  }

  float centered = (static_cast<float>(raw) - center) / 512.0f;
  centered = constrain(centered, -1.0f, 1.0f);
  if (invert) {
    centered = -centered;
  }
  return centered;
}

int16_t normalizedToCommand(float value, int16_t minCmd, int16_t maxCmd) {
  value = constrain(value, 0.0f, 1.0f);
  float scaled = minCmd + value * (maxCmd - minCmd);
  return static_cast<int16_t>(scaled + 0.5f);
}

int16_t centeredToCommand(float value, int16_t amplitude) {
  value = constrain(value, -1.0f, 1.0f);
  float scaled = value * amplitude;
  if (scaled >= 0.0f) {
    return static_cast<int16_t>(scaled + 0.5f);
  } else {
    return static_cast<int16_t>(scaled - 0.5f);
  }
}

uint8_t readButtons() {
  uint8_t mask = 0;
  if (digitalRead(PIN_JOY_LEFT_SEL) == LOW)  mask |= (1 << BTN_JOY_LEFT);
  if (digitalRead(PIN_JOY_RIGHT_SEL) == LOW) mask |= (1 << BTN_JOY_RIGHT);

  if (digitalRead(PIN_BTN_1) == LOW) mask |= (1 << BTN_1);
  if (digitalRead(PIN_BTN_2) == LOW) mask |= (1 << BTN_2);
  if (digitalRead(PIN_BTN_3) == LOW) mask |= (1 << BTN_3);
  if (digitalRead(PIN_BTN_4) == LOW) mask |= (1 << BTN_4);

  if (digitalRead(PIN_SW_1) == LOW) mask |= (1 << BTN_SW1);
  if (digitalRead(PIN_SW_2) == LOW) mask |= (1 << BTN_SW2);

  return mask;
}

void updatePacket() {
  float throttleNorm = readAnalogNormalized(PIN_JOY_LEFT_V, THROTTLE_INVERT);
  float yawCentered = readAnalogCentered(PIN_JOY_LEFT_H, YAW_INVERT);
  float pitchCentered = readAnalogCentered(PIN_JOY_RIGHT_V, PITCH_INVERT);
  float rollCentered = readAnalogCentered(PIN_JOY_RIGHT_H, ROLL_INVERT);

  float pot1Centered = readAnalogCentered(PIN_POT_1, POT1_INVERT);
  float pot2Centered = readAnalogCentered(PIN_POT_2, POT2_INVERT);

  packet.sequence = sequenceCounter++;
  packet.throttle = normalizedToCommand(throttleNorm, 0, 1000);
  packet.roll = applyDeadband(centeredToCommand(rollCentered, 500), STICK_DEADBAND);
  packet.pitch = applyDeadband(centeredToCommand(pitchCentered, 500), STICK_DEADBAND);
  packet.yaw = applyDeadband(centeredToCommand(yawCentered, 500), STICK_DEADBAND);
  packet.aux1 = centeredToCommand(pot1Centered, 500);
  packet.aux2 = centeredToCommand(pot2Centered, 500);
  packet.buttons = readButtons();
  packet.reserved = 0;
  packet.checksum = computeChecksum(packet);
}

void sendPacket() {
  bool ok = radio.write(&packet, sizeof(ControlPacket));
  if (!ok) {
    radio.flush_tx();
  }
  linkOk = ok;
}

void updateStatusLed() {
  uint32_t now = millis();
  if (now - lastBlinkMs >= (linkOk ? 500 : 150)) {
    digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS));
    lastBlinkMs = now;
  }
}

void initPins() {
  pinMode(PIN_BTN_1, INPUT_PULLUP);
  pinMode(PIN_BTN_2, INPUT_PULLUP);
  pinMode(PIN_BTN_3, INPUT_PULLUP);
  pinMode(PIN_BTN_4, INPUT_PULLUP);

  pinMode(PIN_SW_1, INPUT_PULLUP);
  pinMode(PIN_SW_2, INPUT_PULLUP);

  pinMode(PIN_JOY_LEFT_SEL, INPUT_PULLUP);
  pinMode(PIN_JOY_RIGHT_SEL, INPUT_PULLUP);

  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, LOW);
}

void initRadio() {
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(108);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);
  radio.setRetries(4, 9);
  radio.setPayloadSize(sizeof(ControlPacket));
  radio.openWritingPipe(RADIO_ADDRESS);
  radio.stopListening();
}

void calibrateJoysticks() {
  // Optional: hold sticks centered during boot
  const uint8_t samples = 25;
  long sumRoll = 0;
  long sumPitch = 0;
  long sumYaw = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    sumRoll += analogRead(PIN_JOY_RIGHT_H);
    sumPitch += analogRead(PIN_JOY_RIGHT_V);
    sumYaw += analogRead(PIN_JOY_LEFT_H);
    delay(4);
  }

  float rollCenter = sumRoll / static_cast<float>(samples);
  float pitchCenter = sumPitch / static_cast<float>(samples);
  float yawCenter = sumYaw / static_cast<float>(samples);

  rollCenterOffset = rollCenter - 512.0f;
  pitchCenterOffset = pitchCenter - 512.0f;
  yawCenterOffset = yawCenter - 512.0f;
}

void setup() {
  initPins();
  Serial.begin(115200);
  delay(100);
  Serial.println(F("RC transmitter starting..."));

  initRadio();
  calibrateJoysticks();

  packet.magic[0] = 'D';
  packet.magic[1] = 'R';
  packet.version = 1;

  lastTransmitUs = micros();
  Serial.println(F("Transmitter ready. Keep throttle low before arming."));
}

void loop() {
  uint32_t nowUs = micros();
  if (nowUs - lastTransmitUs < TRANSMIT_PERIOD_US) {
    updateStatusLed();
    return;
  }
  lastTransmitUs = nowUs;

  updatePacket();
  sendPacket();

  updateStatusLed();
}
