/**
 * DIY Quadcopter RC Transmitter
 * -----------------------------
 * Target board : Arduino Nano (ATmega328P @16MHz)
 * Radio        : nRF24L01+  (CE=D9, CSN=D10)
 * Inputs       :
 *   Left joystick  - V:A0 (Throttle), H:A1 (Yaw), SEL:A4
 *   Right joystick - V:A2 (Pitch),    H:A3 (Roll), SEL:A5
 *   Buttons        - D4..D7 (active low with pull-ups)
 *   Switches       - SW1:D2, SW2:D3 (active low to GND)
 *   Pots           - POT1:A6, POT2:A7
 *
 * Libraries required:
 * - RF24 by TMRh20
 *
 * The sketch reads all controls, normalises them, and transmits RcPacket updates
 * to the drone flight controller at ~50 Hz.
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

const uint8_t PIN_RF_CE = 9;
const uint8_t PIN_RF_CSN = 10;

const uint8_t PIN_JOYL_THROTTLE = A0;
const uint8_t PIN_JOYL_YAW = A1;
const uint8_t PIN_JOYR_PITCH = A2;
const uint8_t PIN_JOYR_ROLL = A3;
const uint8_t PIN_JOYL_SEL = A4;
const uint8_t PIN_JOYR_SEL = A5;

const uint8_t PIN_BUTTON_1 = 4;
const uint8_t PIN_BUTTON_2 = 5;
const uint8_t PIN_BUTTON_3 = 6;
const uint8_t PIN_BUTTON_4 = 7;

const uint8_t PIN_SWITCH_1 = 2;
const uint8_t PIN_SWITCH_2 = 3;

const uint8_t PIN_POT_1 = A6;
const uint8_t PIN_POT_2 = A7;

RF24 radio(PIN_RF_CE, PIN_RF_CSN);
const byte RADIO_ADDRESS_TX[6] = "DRN1";
const byte RADIO_ADDRESS_RX[6] = "RC01";

template <typename T>
T clamp(T value, T minVal, T maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

struct __attribute__((packed)) RcPacket {
  uint32_t sequence;
  uint16_t throttle;   // 0..1000
  int16_t roll;        // -500..500
  int16_t pitch;       // -500..500
  int16_t yaw;         // -500..500
  int16_t aux1;        // -500..500 (pot 1)
  int16_t aux2;        // -500..500 (pot 2)
  uint8_t buttons;     // bitmask buttons D4..D7
  uint8_t switches;    // bit0:SW1, bit1:SW2, bit2:LeftSel, bit3:RightSel
  uint16_t checksum;
};

uint32_t sequenceCounter = 0;
uint32_t lastSendMillis = 0;

uint16_t computeChecksum(const RcPacket &packet) {
  const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&packet);
  uint16_t sum = 0;
  for (size_t i = 0; i < sizeof(RcPacket) - sizeof(packet.checksum); ++i) {
    sum += ptr[i];
  }
  return sum;
}

int16_t mapAxisToSigned(int rawValue, int deadband = 20) {
  int centered = rawValue - 512;
  if (abs(centered) <= deadband) {
    centered = 0;
  } else if (centered > 0) {
    centered -= deadband;
  } else {
    centered += deadband;
  }
  centered = clamp(centered, -512, 512);
  return static_cast<int16_t>((centered / 512.0f) * 500.0f);
}

uint16_t mapThrottleToUnsigned(int rawValue) {
  rawValue = clamp(rawValue, 0, 1023);
  return static_cast<uint16_t>((rawValue / 1023.0f) * 1000.0f);
}

int16_t mapPotToSigned(int rawValue) {
  // pots are assumed centered at mid-scale; convert to -500..500
  return mapAxisToSigned(rawValue, 0);
}

uint8_t readButtons() {
  uint8_t mask = 0;
  mask |= (!digitalRead(PIN_BUTTON_1)) << 0;
  mask |= (!digitalRead(PIN_BUTTON_2)) << 1;
  mask |= (!digitalRead(PIN_BUTTON_3)) << 2;
  mask |= (!digitalRead(PIN_BUTTON_4)) << 3;
  return mask;
}

uint8_t readSwitches(bool joyLeftPressed, bool joyRightPressed) {
  uint8_t mask = 0;
  mask |= (!digitalRead(PIN_SWITCH_1)) << 0;
  mask |= (!digitalRead(PIN_SWITCH_2)) << 1;
  mask |= (!joyLeftPressed) << 2;
  mask |= (!joyRightPressed) << 3;
  return mask;
}

void initInputs() {
  pinMode(PIN_BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_2, INPUT_PULLUP);
  pinMode(PIN_BUTTON_3, INPUT_PULLUP);
  pinMode(PIN_BUTTON_4, INPUT_PULLUP);
  pinMode(PIN_SWITCH_1, INPUT_PULLUP);
  pinMode(PIN_SWITCH_2, INPUT_PULLUP);
  pinMode(PIN_JOYL_SEL, INPUT_PULLUP);
  pinMode(PIN_JOYR_SEL, INPUT_PULLUP);
}

void initRadio() {
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(115);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);
  radio.openWritingPipe(RADIO_ADDRESS_TX);
  radio.openReadingPipe(1, RADIO_ADDRESS_RX);
  radio.stopListening();
}

void sendPacket() {
  RcPacket packet{};
  packet.sequence = sequenceCounter++;
  packet.throttle = mapThrottleToUnsigned(analogRead(PIN_JOYL_THROTTLE));
  packet.roll = mapAxisToSigned(analogRead(PIN_JOYR_ROLL));
  packet.pitch = mapAxisToSigned(analogRead(PIN_JOYR_PITCH));
  packet.yaw = mapAxisToSigned(analogRead(PIN_JOYL_YAW));
  packet.aux1 = mapPotToSigned(analogRead(PIN_POT_1));
  packet.aux2 = mapPotToSigned(analogRead(PIN_POT_2));

  bool joyLeftPressed = !digitalRead(PIN_JOYL_SEL);
  bool joyRightPressed = !digitalRead(PIN_JOYR_SEL);

  packet.buttons = readButtons();
  packet.switches = readSwitches(joyLeftPressed, joyRightPressed);
  packet.checksum = computeChecksum(packet);

  radio.write(&packet, sizeof(packet));
}

void setup() {
  initInputs();
  initRadio();
  sequenceCounter = 0;
  lastSendMillis = millis();
}

void loop() {
  const uint32_t now = millis();
  if (now - lastSendMillis >= 20) { // ~50 Hz
    lastSendMillis = now;
    sendPacket();
  }
}
