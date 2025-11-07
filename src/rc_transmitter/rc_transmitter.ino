/**
 * RC Transmitter sketch for Arduino Nano
 *
 * Hardware:
 *  - NRF24L01 (CE -> D9, CSN -> D10)
 *  - Joystick axes on A0..A3, press buttons on A4/A5
 *  - Buttons on D4..D7
 *  - Toggle switches on D2, D3 (active LOW to GND)
 *  - Potentiometers on A6, A7
 *
 * Sends stick/button states over NRF24L01 at ~100 Hz.
 */

#include <SPI.h>
#include <RF24.h>

// Pin assignments
constexpr uint8_t PIN_RF24_CE = 9;
constexpr uint8_t PIN_RF24_CSN = 10;

constexpr uint8_t PIN_THROTTLE = A0;
constexpr uint8_t PIN_YAW = A1;
constexpr uint8_t PIN_PITCH = A2;
constexpr uint8_t PIN_ROLL = A3;

constexpr uint8_t PIN_JOYSTICK_LEFT_BTN = A4;
constexpr uint8_t PIN_JOYSTICK_RIGHT_BTN = A5;

constexpr uint8_t PIN_BUTTON_1 = 4;
constexpr uint8_t PIN_BUTTON_2 = 5;
constexpr uint8_t PIN_BUTTON_3 = 6;
constexpr uint8_t PIN_BUTTON_4 = 7;

constexpr uint8_t PIN_SWITCH_1 = 2;
constexpr uint8_t PIN_SWITCH_2 = 3;

constexpr uint8_t PIN_POT_1 = A6;
constexpr uint8_t PIN_POT_2 = A7;

constexpr uint8_t PACKET_VERSION = 1;

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);
const uint64_t RADIO_PIPE = 0xF0F0F0F0D2LL;

struct __attribute__((packed)) ControlPacket {
  uint8_t version;
  uint16_t throttle;  // 1000-2000
  uint16_t yaw;       // 1000-2000
  uint16_t pitch;     // 1000-2000
  uint16_t roll;      // 1000-2000
  uint8_t buttons;    // bitfield
  uint8_t switches;   // bitfield
  uint8_t joystickButtons; // bitfield
  uint16_t pot1;      // 0-1023
  uint16_t pot2;      // 0-1023
  uint16_t checksum;
};

ControlPacket packet;

constexpr uint8_t BUTTON_BIT_1 = 0;
constexpr uint8_t BUTTON_BIT_2 = 1;
constexpr uint8_t BUTTON_BIT_3 = 2;
constexpr uint8_t BUTTON_BIT_4 = 3;

constexpr uint8_t SWITCH_BIT_1 = 0;
constexpr uint8_t SWITCH_BIT_2 = 1;

constexpr uint8_t JOY_BUTTON_LEFT = 0;
constexpr uint8_t JOY_BUTTON_RIGHT = 1;

uint16_t calcChecksum(const ControlPacket &pkt) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&pkt);
  uint16_t sum = 0;
  for (size_t i = 0; i < sizeof(ControlPacket) - sizeof(pkt.checksum); ++i) {
    sum += bytes[i];
  }
  return sum;
}

uint16_t readChannel(uint8_t pin, bool center = true) {
  int value = analogRead(pin);
  if (center) {
    return static_cast<uint16_t>(map(value, 0, 1023, 1000, 2000));
  }
  return static_cast<uint16_t>(map(value, 0, 1023, 1000, 2000));
}

uint16_t readThrottle(uint8_t pin) {
  int value = analogRead(pin);
  return static_cast<uint16_t>(map(value, 0, 1023, 1000, 2000));
}

uint16_t readPot(uint8_t pin) {
  return static_cast<uint16_t>(analogRead(pin));
}

uint8_t readButton(uint8_t pin, bool activeLow = true) {
  uint8_t state = digitalRead(pin);
  return activeLow ? !state : state;
}

void configureInputs() {
  pinMode(PIN_BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_2, INPUT_PULLUP);
  pinMode(PIN_BUTTON_3, INPUT_PULLUP);
  pinMode(PIN_BUTTON_4, INPUT_PULLUP);

  pinMode(PIN_SWITCH_1, INPUT_PULLUP);
  pinMode(PIN_SWITCH_2, INPUT_PULLUP);

  pinMode(PIN_JOYSTICK_LEFT_BTN, INPUT_PULLUP);
  pinMode(PIN_JOYSTICK_RIGHT_BTN, INPUT_PULLUP);
}

void setupRadio() {
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(110);
  radio.setRetries(3, 15);
  radio.openWritingPipe(RADIO_PIPE);
  radio.stopListening();
}

void setup() {
  Serial.begin(115200);
  configureInputs();
  setupRadio();

  packet.version = PACKET_VERSION;
}

void loop() {
  packet.throttle = readThrottle(PIN_THROTTLE);
  packet.yaw = readChannel(PIN_YAW);
  packet.pitch = readChannel(PIN_PITCH);
  packet.roll = readChannel(PIN_ROLL);

  packet.buttons = 0;
  packet.buttons |= (readButton(PIN_BUTTON_1) << BUTTON_BIT_1);
  packet.buttons |= (readButton(PIN_BUTTON_2) << BUTTON_BIT_2);
  packet.buttons |= (readButton(PIN_BUTTON_3) << BUTTON_BIT_3);
  packet.buttons |= (readButton(PIN_BUTTON_4) << BUTTON_BIT_4);

  packet.switches = 0;
  packet.switches |= (readButton(PIN_SWITCH_1) << SWITCH_BIT_1);
  packet.switches |= (readButton(PIN_SWITCH_2) << SWITCH_BIT_2);

  packet.joystickButtons = 0;
  packet.joystickButtons |= (readButton(PIN_JOYSTICK_LEFT_BTN) << JOY_BUTTON_LEFT);
  packet.joystickButtons |= (readButton(PIN_JOYSTICK_RIGHT_BTN) << JOY_BUTTON_RIGHT);

  packet.pot1 = readPot(PIN_POT_1);
  packet.pot2 = readPot(PIN_POT_2);

  packet.checksum = calcChecksum(packet);

  radio.write(&packet, sizeof(packet));

  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastPrint > 500) {
    lastPrint = now;
    Serial.print(F("Throttle: "));
    Serial.print(packet.throttle);
    Serial.print(F(" Pitch: "));
    Serial.print(packet.pitch);
    Serial.print(F(" Roll: "));
    Serial.print(packet.roll);
    Serial.print(F(" Yaw: "));
    Serial.print(packet.yaw);
    Serial.print(F(" Buttons: 0b"));
    Serial.println(packet.buttons, BIN);
  }

  delay(10);  // ~100 Hz update rate
}
