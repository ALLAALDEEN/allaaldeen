#include <SPI.h>
#include <RF24.h>

// ----------------------- Pin assignments -----------------------
// Radio
constexpr uint8_t PIN_RF_CE = 9;
constexpr uint8_t PIN_RF_CSN = 10;

// Joysticks (analog axes)
constexpr uint8_t PIN_LEFT_VERT = A0;   // Throttle
constexpr uint8_t PIN_LEFT_HORIZ = A1;  // Yaw
constexpr uint8_t PIN_RIGHT_VERT = A2;  // Pitch
constexpr uint8_t PIN_RIGHT_HORIZ = A3; // Roll

// Joystick push-buttons (active LOW)
constexpr uint8_t PIN_LEFT_PUSH = A4;
constexpr uint8_t PIN_RIGHT_PUSH = A5;

// Momentary buttons (active LOW with pull-ups)
constexpr uint8_t PIN_BUTTON_1 = 4;
constexpr uint8_t PIN_BUTTON_2 = 5;
constexpr uint8_t PIN_BUTTON_3 = 6;
constexpr uint8_t PIN_BUTTON_4 = 7;

// Toggle switches (SPDT to GND, active LOW)
constexpr uint8_t PIN_SWITCH_1 = 2;
constexpr uint8_t PIN_SWITCH_2 = 3;

// Auxiliary potentiometers
constexpr uint8_t PIN_POT_1 = A6;
constexpr uint8_t PIN_POT_2 = A7;

constexpr uint8_t PIN_STATUS_LED = 13;

// ----------------------- Radio configuration -----------------------
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
const byte RADIO_PIPE[6] = "QFC01"; // Must match flight controller

struct ControlPacket {
  uint16_t throttle; // 0-1023
  uint16_t roll;     // 0-1023
  uint16_t pitch;    // 0-1023
  uint16_t yaw;      // 0-1023
  uint8_t buttons;   // bit0..3 buttons, bit4 left stick push, bit5 right stick push
  uint8_t switches;  // bit0 SW1, bit1 SW2
  uint16_t pot1;     // 0-1023
  uint16_t pot2;     // 0-1023
  uint32_t sequence; // packet counter
};

ControlPacket txPacket{};
uint32_t packetCounter = 0;
unsigned long lastSendMs = 0;
bool linkOk = false;

// Simple calibration for joystick centres
struct AxisCal {
  uint16_t minValue = 100;
  uint16_t centreValue = 512;
  uint16_t maxValue = 900;
};

AxisCal calThrottle{80, 520, 940};
AxisCal calYaw{80, 512, 940};
AxisCal calPitch{80, 520, 940};
AxisCal calRoll{80, 512, 940};

void applyDeadband(uint16_t& value, uint16_t centre, uint8_t deadband = 4) {
  if (value + deadband >= centre && value <= centre + deadband) {
    value = centre;
  }
}

uint16_t readAxis(uint8_t analogPin, AxisCal& cal, bool invert = false) {
  uint16_t raw = analogRead(analogPin);

  // Capture extrema for adaptive calibration
  if (raw < cal.minValue) cal.minValue = raw;
  if (raw > cal.maxValue) cal.maxValue = raw;

  uint16_t value;
  if (raw >= cal.centreValue) {
    value = map(raw, cal.centreValue, cal.maxValue, 512, 1023);
  } else {
    value = map(raw, cal.minValue, cal.centreValue, 0, 512);
  }
  applyDeadband(value, 512);
  value = constrain(value, 0, 1023);
  if (invert) {
    value = 1023 - value;
  }
  return value;
}

uint8_t readButtons() {
  uint8_t bits = 0;
  bits |= (digitalRead(PIN_BUTTON_1) == LOW) ? (1 << 0) : 0;
  bits |= (digitalRead(PIN_BUTTON_2) == LOW) ? (1 << 1) : 0;
  bits |= (digitalRead(PIN_BUTTON_3) == LOW) ? (1 << 2) : 0;
  bits |= (digitalRead(PIN_BUTTON_4) == LOW) ? (1 << 3) : 0;
  bits |= (digitalRead(PIN_LEFT_PUSH) == LOW) ? (1 << 4) : 0;
  bits |= (digitalRead(PIN_RIGHT_PUSH) == LOW) ? (1 << 5) : 0;
  return bits;
}

uint8_t readSwitches() {
  uint8_t bits = 0;
  bits |= (digitalRead(PIN_SWITCH_1) == LOW) ? (1 << 0) : 0;
  bits |= (digitalRead(PIN_SWITCH_2) == LOW) ? (1 << 1) : 0;
  return bits;
}

void setupInputs() {
  pinMode(PIN_BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_2, INPUT_PULLUP);
  pinMode(PIN_BUTTON_3, INPUT_PULLUP);
  pinMode(PIN_BUTTON_4, INPUT_PULLUP);

  pinMode(PIN_SWITCH_1, INPUT_PULLUP);
  pinMode(PIN_SWITCH_2, INPUT_PULLUP);

  pinMode(PIN_LEFT_PUSH, INPUT_PULLUP);
  pinMode(PIN_RIGHT_PUSH, INPUT_PULLUP);
}

void setupRadio() {
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(90);
  radio.setAutoAck(true);
  radio.setRetries(3, 5);
  radio.enableDynamicPayloads();
  radio.openWritingPipe(RADIO_PIPE);
  radio.stopListening();
}

void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  setupInputs();

  Serial.begin(115200);
  while (!Serial) {
    delay(5);
  }
  Serial.println(F("RC Transmitter Boot"));

  setupRadio();
  Serial.println(F("RF24 ready"));

  // Quick centre calibration
  const uint16_t samples = 200;
  uint32_t sumThrottle = 0, sumYaw = 0, sumPitch = 0, sumRoll = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    sumThrottle += analogRead(PIN_LEFT_VERT);
    sumYaw += analogRead(PIN_LEFT_HORIZ);
    sumPitch += analogRead(PIN_RIGHT_VERT);
    sumRoll += analogRead(PIN_RIGHT_HORIZ);
    delay(2);
  }
  calThrottle.centreValue = sumThrottle / samples;
  calYaw.centreValue = sumYaw / samples;
  calPitch.centreValue = sumPitch / samples;
  calRoll.centreValue = sumRoll / samples;

  lastSendMs = millis();
}

void updateStatusLed(bool link) {
  static unsigned long lastBlink = 0;
  if (link) {
    digitalWrite(PIN_STATUS_LED, HIGH);
  } else {
    unsigned long now = millis();
    if (now - lastBlink > 250) {
      digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
      lastBlink = now;
    }
  }
}

void collectInputs(ControlPacket& packet) {
  packet.throttle = readAxis(PIN_LEFT_VERT, calThrottle, true); // invert so stick up = higher value
  packet.yaw = readAxis(PIN_LEFT_HORIZ, calYaw, false);
  packet.pitch = readAxis(PIN_RIGHT_VERT, calPitch, true); // stick forward => more pitch forward
  packet.roll = readAxis(PIN_RIGHT_HORIZ, calRoll, false);

  packet.buttons = readButtons();
  packet.switches = readSwitches();

  packet.pot1 = analogRead(PIN_POT_1);
  packet.pot2 = analogRead(PIN_POT_2);
  packet.sequence = packetCounter++;
}

void loop() {
  const unsigned long now = millis();
  if (now - lastSendMs >= 20) { // 50 Hz update
    collectInputs(txPacket);
    bool ok = radio.write(&txPacket, sizeof(txPacket));
    linkOk = ok;
    if (!ok) {
      Serial.println(F("Radio write failed"));
    }
    updateStatusLed(linkOk);
    lastSendMs = now;
  }
}
