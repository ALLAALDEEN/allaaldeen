/**
 * RC_Transmitter.ino
 *
 * Arduino Nano based RC transmitter for quadcopter project.
 * Hardware:
 *  - NRF24L01 (CE -> D9, CSN -> D10)
 *  - Left Joystick:  V -> A0 (Throttle), H -> A1 (Yaw), SEL -> A4
 *  - Right Joystick: V -> A2 (Pitch),    H -> A3 (Roll), SEL -> A5
 *  - Buttons: D4-D7, Toggle switches: D2, D3
 *  - Potentiometers: A6, A7
 *
 * Joystick channels are mapped to standard RC channel ranges (1000-2000us).
 * Button and switch states are encoded into bitfields.
 */

#include <SPI.h>
#include <RF24.h>

// NRF24 connections
constexpr uint8_t PIN_RADIO_CE  = 9;
constexpr uint8_t PIN_RADIO_CSN = 10;

// Joysticks
constexpr uint8_t PIN_THROTTLE = A0;
constexpr uint8_t PIN_YAW      = A1;
constexpr uint8_t PIN_PITCH    = A2;
constexpr uint8_t PIN_ROLL     = A3;
constexpr uint8_t PIN_JOY_LEFT_CLICK  = A4;
constexpr uint8_t PIN_JOY_RIGHT_CLICK = A5;

// Potentiometers
constexpr uint8_t PIN_POT_1 = A6;
constexpr uint8_t PIN_POT_2 = A7;

// Buttons (active low when wired to GND)
constexpr uint8_t PIN_BUTTON_1 = 4;
constexpr uint8_t PIN_BUTTON_2 = 5;
constexpr uint8_t PIN_BUTTON_3 = 6;
constexpr uint8_t PIN_BUTTON_4 = 7;

// Toggle switches (active low when connected to GND)
constexpr uint8_t PIN_SWITCH_1 = 2;
constexpr uint8_t PIN_SWITCH_2 = 3;

// Optional heartbeat LED (built-in LED on D13)
constexpr uint8_t PIN_LED = LED_BUILTIN;

constexpr uint32_t RADIO_UPDATE_HZ = 100;
constexpr uint32_t RADIO_PERIOD_MS = 1000UL / RADIO_UPDATE_HZ;

// Payload definition (packed to avoid padding differences)
struct __attribute__((packed)) RadioPacket {
  uint16_t throttle;
  uint16_t yaw;
  uint16_t pitch;
  uint16_t roll;
  uint16_t aux1;     // Left joystick click
  uint16_t aux2;     // Right joystick click
  uint16_t pot1;
  uint16_t pot2;
  uint8_t buttons;   // bit0: BTN1 ... bit3: BTN4
  uint8_t switches;  // bit0: SW1, bit1: SW2
  uint16_t checksum; // simple XOR checksum
};

RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);
const byte RADIO_ADDRESS[6] = "FC01";

uint32_t lastSendMs = 0;

uint16_t mapAnalogToRC(int value, bool invert = false) {
  value = constrain(value, 0, 1023);
  if (invert) {
    value = 1023 - value;
  }
  return static_cast<uint16_t>(map(value, 0, 1023, 1000, 2000));
}

uint8_t readButton(uint8_t pin) {
  return digitalRead(pin) == LOW ? 1 : 0;
}

RadioPacket buildPacket() {
  RadioPacket packet;

  packet.throttle = mapAnalogToRC(analogRead(PIN_THROTTLE));
  packet.yaw      = mapAnalogToRC(analogRead(PIN_YAW));
  packet.pitch    = mapAnalogToRC(analogRead(PIN_PITCH), true); // invert for natural stick feel
  packet.roll     = mapAnalogToRC(analogRead(PIN_ROLL));
  packet.aux1     = mapAnalogToRC(analogRead(PIN_JOY_LEFT_CLICK));
  packet.aux2     = mapAnalogToRC(analogRead(PIN_JOY_RIGHT_CLICK));
  packet.pot1     = mapAnalogToRC(analogRead(PIN_POT_1));
  packet.pot2     = mapAnalogToRC(analogRead(PIN_POT_2));

  packet.buttons = (readButton(PIN_BUTTON_1) << 0) |
                   (readButton(PIN_BUTTON_2) << 1) |
                   (readButton(PIN_BUTTON_3) << 2) |
                   (readButton(PIN_BUTTON_4) << 3);

  packet.switches = (readButton(PIN_SWITCH_1) << 0) | // reuse readButton (active low)
                    (readButton(PIN_SWITCH_2) << 1);

  // simple checksum: XOR of all 16-bit fields and switch/button bytes
  uint16_t checksum = 0xFFFF;
  checksum ^= packet.throttle;
  checksum ^= packet.yaw;
  checksum ^= packet.pitch;
  checksum ^= packet.roll;
  checksum ^= packet.aux1;
  checksum ^= packet.aux2;
  checksum ^= packet.pot1;
  checksum ^= packet.pot2;
  checksum ^= packet.buttons;
  checksum ^= packet.switches;
  packet.checksum = checksum;

  return packet;
}

void setupPins() {
  pinMode(PIN_BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_2, INPUT_PULLUP);
  pinMode(PIN_BUTTON_3, INPUT_PULLUP);
  pinMode(PIN_BUTTON_4, INPUT_PULLUP);
  pinMode(PIN_SWITCH_1, INPUT_PULLUP);
  pinMode(PIN_SWITCH_2, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
}

void setupRadio() {
  if (!radio.begin()) {
    // If radio fails to initialize, blink rapidly to alert the pilot.
    while (true) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(100);
    }
  }
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(108);
  radio.setRetries(3, 15); // delay, count
  radio.openWritingPipe(RADIO_ADDRESS);
  radio.stopListening();
}

void setup() {
  Serial.begin(115200);
  setupPins();
  setupRadio();
  lastSendMs = millis();
}

void loop() {
  const uint32_t now = millis();
  if (now - lastSendMs >= RADIO_PERIOD_MS) {
    lastSendMs = now;

    RadioPacket packet = buildPacket();
    bool ok = radio.write(&packet, sizeof(packet));

    digitalWrite(PIN_LED, ok ? HIGH : LOW);

    // Optional telemetry over serial for setup/calibration
    if (Serial) {
      Serial.print(F("T:"));
      Serial.print(packet.throttle);
      Serial.print(F(" Y:"));
      Serial.print(packet.yaw);
      Serial.print(F(" P:"));
      Serial.print(packet.pitch);
      Serial.print(F(" R:"));
      Serial.print(packet.roll);
      Serial.print(F(" B:"));
      Serial.print(packet.buttons, BIN);
      Serial.print(F(" S:"));
      Serial.print(packet.switches, BIN);
      Serial.print(F(" -> "));
      Serial.println(ok ? F("OK") : F("FAIL"));
    }
  }
}
