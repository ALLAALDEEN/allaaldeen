#include <SPI.h>
#include <EEPROM.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// ---------------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------------
constexpr uint8_t PIN_JOYSTICK_YAW = A0;
constexpr uint8_t PIN_JOYSTICK_THROTTLE = A1;
constexpr uint8_t PIN_JOYSTICK_ROLL = A2;
constexpr uint8_t PIN_JOYSTICK_PITCH = A3;

constexpr uint8_t PIN_SWITCH_ARM = 2;      // Kill/Arm toggle (INPUT_PULLUP)
constexpr uint8_t PIN_BUTTON_CAL = 3;      // Button 1
constexpr uint8_t PIN_BUTTON_MOTOR = 4;    // Button 2

constexpr uint8_t PIN_LCD_SCLK = 13;
constexpr uint8_t PIN_LCD_DIN = 11;
constexpr uint8_t PIN_LCD_DC = 9;
constexpr uint8_t PIN_LCD_CS = 10;
constexpr uint8_t PIN_LCD_RST = 8;
constexpr uint8_t PIN_LCD_BACKLIGHT = 5;

constexpr uint8_t PIN_RADIO_CE = 6;
constexpr uint8_t PIN_RADIO_CSN = 7;

constexpr uint8_t PIN_STATUS_LED = A4; // can act as digital 18
constexpr uint8_t PIN_BUZZER = A5;

// ---------------------------------------------------------------------------
// NRF24 configuration
// ---------------------------------------------------------------------------
constexpr byte RADIO_ADDRESS[6] = "FC001";
constexpr byte RADIO_ADDRESS_BACK[6] = "TX001";
constexpr uint8_t RADIO_CHANNEL = 110;
constexpr uint8_t RADIO_PAYLOAD_SIZE = 32;

// ---------------------------------------------------------------------------
// ESC limits (mirrors FC)
// ---------------------------------------------------------------------------
constexpr uint16_t ESC_MIN_US = 1000;
constexpr uint16_t ESC_IDLE_US = 1120;
constexpr uint16_t ESC_MAX_US = 2000;

// ---------------------------------------------------------------------------
// Joystick calibration storage
// ---------------------------------------------------------------------------
struct AxisCalibration {
  uint16_t min;
  uint16_t center;
  uint16_t max;
};

struct JoystickCalibration {
  AxisCalibration roll;
  AxisCalibration pitch;
  AxisCalibration yaw;
  AxisCalibration throttle;
  uint32_t crc;
};

JoystickCalibration joyCal;

constexpr uint16_t AXIS_DEADBAND = 18; // ~2% deadband

constexpr int EEPROM_SIGNATURE_ADDR = 0;
constexpr int EEPROM_CAL_ADDR = 4;

// ---------------------------------------------------------------------------
// RF packets (must mirror FC definitions)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) RadioCommand {
  uint8_t flags;
  uint16_t throttle;
  int16_t roll;
  int16_t pitch;
  int16_t yawRate;
  uint16_t seq;
  uint16_t checksum;
};

struct __attribute__((packed)) TelemetryFrame {
  uint8_t state;
  int16_t rollDeg;
  int16_t pitchDeg;
  int16_t yawDeg;
  int16_t altitudeCm;
  uint16_t batteryMv;
  uint16_t linkQuality;
  uint16_t lastSeq;
  uint16_t checksum;
};

// Flag bit definitions (shared with FC)
constexpr uint8_t FLAG_ARMED = 0x01;
constexpr uint8_t FLAG_KILL = 0x02;
constexpr uint8_t FLAG_CALIBRATE = 0x04;
constexpr uint8_t FLAG_MOTOR_SPIN = 0x08;

// ---------------------------------------------------------------------------
// UI state machine
// ---------------------------------------------------------------------------
enum class UiState : uint8_t {
  STARTUP = 0,
  WAIT_LINK,
  REQUIRE_KILL,
  REQUEST_CALIBRATION,
  CALIBRATING,
  READY_TO_ARM,
  MOTOR_SPIN_PROMPT,
  FLYING,
  LOST_LINK
};

UiState uiState = UiState::STARTUP;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);
Adafruit_PCD8544 display = Adafruit_PCD8544(PIN_LCD_SCLK, PIN_LCD_DIN, PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_RST);

TelemetryFrame telemetry = {};
RadioCommand commandFrame = {};

uint16_t seqCounter = 1;
uint32_t lastAckMillis = 0;
uint32_t lastHeartbeatMillis = 0;

bool linkActive = false;
bool pendingCalibrateCommand = false;
bool pendingMotorSpinCommand = false;

bool joystickCalValid = false;
bool killSwitchLatched = true;

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------
uint32_t crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; ++i) {
    uint8_t byte = data[i];
    crc ^= byte;
    for (uint8_t j = 0; j < 8; ++j) {
      uint32_t mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
  }
  return ~crc;
}

uint16_t checksum16(const uint8_t* data, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; ++i) {
    sum += data[i];
  }
  sum = (sum & 0xFFFF) + (sum >> 16);
  sum = (sum & 0xFFFF) + (sum >> 16);
  return static_cast<uint16_t>(~sum);
}

void setStatusLed(bool on) {
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

void pulseBuzzer(uint8_t repeats, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < repeats; ++i) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(onMs);
    digitalWrite(PIN_BUZZER, LOW);
    delay(offMs);
  }
}

// ---------------------------------------------------------------------------
// EEPROM handling
// ---------------------------------------------------------------------------
bool loadJoystickCalibration() {
  uint32_t signature = 0;
  EEPROM.get(EEPROM_SIGNATURE_ADDR, signature);
  if (signature != 0xA5C0DE55) {
    return false;
  }

  EEPROM.get(EEPROM_CAL_ADDR, joyCal);
  uint32_t storedCrc = joyCal.crc;
  joyCal.crc = 0;
  if (crc32(reinterpret_cast<uint8_t*>(&joyCal), sizeof(JoystickCalibration)) != storedCrc) {
    return false;
  }
  joyCal.crc = storedCrc;
  return true;
}

void saveJoystickCalibration() {
  joyCal.crc = 0;
  joyCal.crc = crc32(reinterpret_cast<uint8_t*>(&joyCal), sizeof(JoystickCalibration));
  EEPROM.put(EEPROM_CAL_ADDR, joyCal);
  uint32_t signature = 0xA5C0DE55;
  EEPROM.put(EEPROM_SIGNATURE_ADDR, signature);
}

// ---------------------------------------------------------------------------
// Joystick calibration routine
// ---------------------------------------------------------------------------
uint16_t readAxis(uint8_t pin) {
  return analogRead(pin);
}

void showCenteredText(const char* title, const char* line2 = nullptr, const char* line3 = nullptr) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(title);
  if (line2) {
    display.println(line2);
  }
  if (line3) {
    display.println(line3);
  }
  display.display();
}

void waitForButtonRelease(uint8_t pin) {
  while (digitalRead(pin) == LOW) {
    delay(10);
  }
}

void captureCenterValues() {
  const uint16_t samples = 300;
  uint32_t rollSum = 0, pitchSum = 0, yawSum = 0, thrSum = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    rollSum += readAxis(PIN_JOYSTICK_ROLL);
    pitchSum += readAxis(PIN_JOYSTICK_PITCH);
    yawSum += readAxis(PIN_JOYSTICK_YAW);
    thrSum += readAxis(PIN_JOYSTICK_THROTTLE);
    delay(5);
  }
  joyCal.roll.center = rollSum / samples;
  joyCal.pitch.center = pitchSum / samples;
  joyCal.yaw.center = yawSum / samples;
  joyCal.throttle.center = thrSum / samples;
}

void captureMinMaxValues() {
  joyCal.roll.min = joyCal.roll.max = readAxis(PIN_JOYSTICK_ROLL);
  joyCal.pitch.min = joyCal.pitch.max = readAxis(PIN_JOYSTICK_PITCH);
  joyCal.yaw.min = joyCal.yaw.max = readAxis(PIN_JOYSTICK_YAW);
  joyCal.throttle.min = joyCal.throttle.max = readAxis(PIN_JOYSTICK_THROTTLE);

  uint32_t start = millis();
  while (millis() - start < 8000) {
    joyCal.roll.min = min<uint16_t>(joyCal.roll.min, readAxis(PIN_JOYSTICK_ROLL));
    joyCal.roll.max = max<uint16_t>(joyCal.roll.max, readAxis(PIN_JOYSTICK_ROLL));
    joyCal.pitch.min = min<uint16_t>(joyCal.pitch.min, readAxis(PIN_JOYSTICK_PITCH));
    joyCal.pitch.max = max<uint16_t>(joyCal.pitch.max, readAxis(PIN_JOYSTICK_PITCH));
    joyCal.yaw.min = min<uint16_t>(joyCal.yaw.min, readAxis(PIN_JOYSTICK_YAW));
    joyCal.yaw.max = max<uint16_t>(joyCal.yaw.max, readAxis(PIN_JOYSTICK_YAW));
    joyCal.throttle.min = min<uint16_t>(joyCal.throttle.min, readAxis(PIN_JOYSTICK_THROTTLE));
    joyCal.throttle.max = max<uint16_t>(joyCal.throttle.max, readAxis(PIN_JOYSTICK_THROTTLE));
    delay(5);
  }
}

bool axisValid(const AxisCalibration& axis) {
  return axis.max > axis.min && (axis.center > axis.min) && (axis.center < axis.max);
}

void ensureCalibrationBounds() {
  if (!axisValid(joyCal.roll) || !axisValid(joyCal.pitch) || !axisValid(joyCal.yaw)) {
    joystickCalValid = false;
    return;
  }
  if (joyCal.throttle.max <= joyCal.throttle.min + 10) {
    joystickCalValid = false;
    return;
  }
  joystickCalValid = true;
}

void runJoystickCalibrationWizard() {
  showCenteredText("Joystick Cal", "Press Btn2", "to start");
  while (digitalRead(PIN_BUTTON_MOTOR) == HIGH) {
    delay(10);
  }
  waitForButtonRelease(PIN_BUTTON_MOTOR);

  showCenteredText("Center sticks", "Hold center", "Btn2=OK");
  captureCenterValues();

  showCenteredText("Sweep sticks", "Move all axes", "8s...");
  captureMinMaxValues();

  showCenteredText("Throttle LOW", "Set min & hold", "Btn2=OK");
  waitForButtonRelease(PIN_BUTTON_MOTOR);

  joyCal.throttle.min = readAxis(PIN_JOYSTICK_THROTTLE);
  delay(200);
  joyCal.throttle.min = (joyCal.throttle.min + readAxis(PIN_JOYSTICK_THROTTLE)) / 2;

  ensureCalibrationBounds();
  if (joystickCalValid) {
    saveJoystickCalibration();
    pulseBuzzer(2, 80, 80);
    showCenteredText("Cal saved", "Release sticks");
    delay(1200);
  } else {
    pulseBuzzer(3, 120, 80);
    showCenteredText("Cal failed", "Retry!");
    delay(1500);
  }
}

// ---------------------------------------------------------------------------
// Axis scaling helpers
// ---------------------------------------------------------------------------
int16_t mapAxisToSigned(const AxisCalibration& axis, uint16_t raw, int16_t scale) {
  if (raw >= axis.center) {
    uint16_t span = max<uint16_t>(axis.max - axis.center, 1);
    long value = (long)(raw - axis.center) * scale / span;
    return (int16_t)min<long>(value, scale);
  } else {
    uint16_t span = max<uint16_t>(axis.center - axis.min, 1);
    long value = (long)(raw - axis.center) * scale / span;
    return (int16_t)max<long>(value, -scale);
  }
}

int16_t applyDeadband(int16_t value, int16_t deadband) {
  if (abs(value) <= deadband) {
    return 0;
  }
  if (value > 0) {
    return value - deadband;
  }
  return value + deadband;
}

uint16_t mapThrottleToUs(uint16_t raw) {
  uint16_t clamped = constrain(raw, joyCal.throttle.min, joyCal.throttle.max);
  long span = max<long>(joyCal.throttle.max - joyCal.throttle.min, 1);
  long value = ESC_MIN_US + (long)(clamped - joyCal.throttle.min) * (ESC_MAX_US - ESC_MIN_US) / span;
  return constrain((int)value, ESC_MIN_US, ESC_MAX_US);
}

// ---------------------------------------------------------------------------
// UI rendering
// ---------------------------------------------------------------------------
const char* flightStateName(uint8_t state) {
  switch (state) {
    case 0: return "POWER_ON";
    case 1: return "LINK_WAIT";
    case 2: return "SAFE_KILL";
    case 3: return "CALIBRING";
    case 4: return "READY";
    case 5: return "MOTOR";
    case 6: return "ARMED";
    default: return "UNKNOWN";
  }
}

void renderStatusFrame() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print("St: ");
  display.println(flightStateName(telemetry.state));
  display.print("Thr:");
  display.print((commandFrame.throttle - ESC_MIN_US) * 100 / (ESC_MAX_US - ESC_MIN_US));
  display.print("% ");
  display.print("Yaw:");
  display.println(telemetry.yawDeg / 10.0f, 1);
  display.print("Rol:");
  display.print(telemetry.rollDeg / 10.0f, 1);
  display.print(" Pit:");
  display.println(telemetry.pitchDeg / 10.0f, 1);
  display.print("Alt:");
  display.print(telemetry.altitudeCm / 100.0f, 1);
  display.print("m ");
  display.print("LQ:");
  display.println(telemetry.linkQuality);
  display.display();
}

void renderWizard(const char* line1, const char* line2 = nullptr, const char* line3 = nullptr) {
  showCenteredText(line1, line2, line3);
}

// ---------------------------------------------------------------------------
// Input helpers
// ---------------------------------------------------------------------------
bool readButton(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

bool killSwitchEngaged() {
  // Assuming toggle to GND = KILL (LOW)
  return digitalRead(PIN_SWITCH_ARM) == LOW;
}

// ---------------------------------------------------------------------------
// Radio communication
// ---------------------------------------------------------------------------
bool sendCommand() {
  commandFrame.seq = seqCounter++;
  commandFrame.checksum = 0;
  commandFrame.checksum = checksum16(reinterpret_cast<uint8_t*>(&commandFrame), sizeof(RadioCommand));

  radio.stopListening();
  bool ok = radio.write(&commandFrame, sizeof(RadioCommand));
  radio.startListening();

  if (ok) {
    lastHeartbeatMillis = millis();
  }
  return ok;
}

bool readTelemetry() {
  if (!radio.available()) {
    return false;
  }
  radio.read(&telemetry, sizeof(TelemetryFrame));
  uint16_t receivedChecksum = telemetry.checksum;
  telemetry.checksum = 0;
  if (checksum16(reinterpret_cast<uint8_t*>(&telemetry), sizeof(TelemetryFrame)) != receivedChecksum) {
    return false;
  }
  telemetry.checksum = receivedChecksum;
  lastAckMillis = millis();
  linkActive = true;
  setStatusLed(!digitalRead(PIN_STATUS_LED));
  return true;
}

bool linkTimedOut() {
  return millis() - lastAckMillis > 500;
}

// ---------------------------------------------------------------------------
// Control logic
// ---------------------------------------------------------------------------
void updateUiStateMachine() {
  switch (uiState) {
    case UiState::STARTUP:
      renderWizard("Booting...", "Waiting RF");
      uiState = UiState::WAIT_LINK;
      break;

    case UiState::WAIT_LINK:
      renderWizard("NRF Link", linkActive ? "Connected" : "Searching");
      if (linkActive) {
        pulseBuzzer(1, 100, 80);
        uiState = UiState::REQUIRE_KILL;
      }
      break;

    case UiState::REQUIRE_KILL:
      renderWizard("Toggle KILL", "Push switch up");
      if (killSwitchEngaged()) {
        pulseBuzzer(2, 70, 80);
        uiState = UiState::REQUEST_CALIBRATION;
      }
      break;

    case UiState::REQUEST_CALIBRATION:
      renderWizard("Press Btn1", "Calibrate IMU", "and ESC");
      if (readButton(PIN_BUTTON_CAL)) {
        pendingCalibrateCommand = true;
        uiState = UiState::CALIBRATING;
        pulseBuzzer(1, 160, 100);
      }
      break;

    case UiState::CALIBRATING:
      renderWizard("Calibrating...", "Keep still");
      if (telemetry.state == 4 || telemetry.state == 6) { // READY or ARMED
        uiState = UiState::READY_TO_ARM;
        pendingCalibrateCommand = false;
        pulseBuzzer(2, 60, 60);
      }
      break;

    case UiState::READY_TO_ARM:
      renderWizard("Toggle ARM", "Switch down");
      if (!killSwitchEngaged()) {
        uiState = UiState::MOTOR_SPIN_PROMPT;
        pulseBuzzer(1, 120, 120);
      }
      break;

    case UiState::MOTOR_SPIN_PROMPT:
      renderWizard("Press Btn2", "Motor soft spin");
      if (readButton(PIN_BUTTON_MOTOR)) {
        pendingMotorSpinCommand = true;
      } else if (pendingMotorSpinCommand) {
        // release indicates done
        pendingMotorSpinCommand = false;
        uiState = UiState::FLYING;
        pulseBuzzer(3, 40, 60);
      }
      break;

    case UiState::FLYING:
      renderStatusFrame();
      if (killSwitchEngaged()) {
        uiState = UiState::REQUIRE_KILL;
      } else if (linkTimedOut()) {
        uiState = UiState::LOST_LINK;
        setStatusLed(false);
      }
      return;

    case UiState::LOST_LINK:
      renderWizard("Link lost!", "Check power");
      if (!linkTimedOut()) {
        uiState = UiState::REQUIRE_KILL;
      }
      break;
  }
}

void buildCommandFrame() {
  commandFrame.flags = 0;

  bool kill = killSwitchEngaged();
  if (kill) {
    commandFrame.flags |= FLAG_KILL;
  }

  if (uiState == UiState::FLYING || uiState == UiState::MOTOR_SPIN_PROMPT || uiState == UiState::READY_TO_ARM) {
    commandFrame.flags |= FLAG_ARMED;
  }

  if (pendingCalibrateCommand) {
    commandFrame.flags |= FLAG_CALIBRATE;
  }
  if (pendingMotorSpinCommand) {
    commandFrame.flags |= FLAG_MOTOR_SPIN;
  }

  if (!joystickCalValid) {
    commandFrame.throttle = ESC_MIN_US;
    commandFrame.roll = 0;
    commandFrame.pitch = 0;
    commandFrame.yawRate = 0;
    return;
  }

  uint16_t throttleRaw = readAxis(PIN_JOYSTICK_THROTTLE);
  uint16_t throttleUs = kill ? ESC_MIN_US : mapThrottleToUs(throttleRaw);

  int16_t rollCmd = applyDeadband(mapAxisToSigned(joyCal.roll, readAxis(PIN_JOYSTICK_ROLL), 300), AXIS_DEADBAND);
  int16_t pitchCmd = applyDeadband(mapAxisToSigned(joyCal.pitch, readAxis(PIN_JOYSTICK_PITCH), 300), AXIS_DEADBAND);
  int16_t yawCmd = applyDeadband(mapAxisToSigned(joyCal.yaw, readAxis(PIN_JOYSTICK_YAW), 400), AXIS_DEADBAND);

  commandFrame.throttle = throttleUs;
  commandFrame.roll = rollCmd * 10 / 3;   // scale to tenths of degrees (~±100)
  commandFrame.pitch = pitchCmd * 10 / 3;
  commandFrame.yawRate = yawCmd * 2;      // convert to deg/s tenths
}

// ---------------------------------------------------------------------------
// Setup & loop
// ---------------------------------------------------------------------------
void setup() {
  pinMode(PIN_SWITCH_ARM, INPUT_PULLUP);
  pinMode(PIN_BUTTON_CAL, INPUT_PULLUP);
  pinMode(PIN_BUTTON_MOTOR, INPUT_PULLUP);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  display.begin();
  display.setContrast(55);
  analogWrite(PIN_LCD_BACKLIGHT, 180);

  Serial.begin(115200);
  Serial.println(F("RC Transmitter Booting..."));

  joystickCalValid = loadJoystickCalibration();
  if (!joystickCalValid) {
    Serial.println(F("No joystick calibration found. Starting wizard."));
    runJoystickCalibrationWizard();
    joystickCalValid = loadJoystickCalibration();
  }
  if (joystickCalValid) {
    ensureCalibrationBounds();
  }
  if (!joystickCalValid) {
    Serial.println(F("Calibration invalid - sticks disabled until recalibrated."));
  }

  radio.begin();
  radio.setChannel(RADIO_CHANNEL);
  radio.setPayloadSize(RADIO_PAYLOAD_SIZE);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(3, 5);
  radio.openWritingPipe(RADIO_ADDRESS);
  radio.openReadingPipe(1, RADIO_ADDRESS_BACK);
  radio.startListening();

  uiState = UiState::STARTUP;
  setStatusLed(true);
  pulseBuzzer(1, 80, 80);
}

void loop() {
  readTelemetry();

  if (linkActive && linkTimedOut()) {
    linkActive = false;
    uiState = UiState::LOST_LINK;
    pulseBuzzer(3, 80, 60);
  }

  updateUiStateMachine();

  if (!joystickCalValid) {
    return;
  }

  buildCommandFrame();
  sendCommand();

  if (millis() - lastHeartbeatMillis > 200) {
    setStatusLed(false);
  }
}
