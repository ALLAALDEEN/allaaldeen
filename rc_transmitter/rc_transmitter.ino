/**
 * RC Transmitter Firmware for Quadcopter
 *
 * Hardware:
 *  - Arduino Nano
 *  - Two-axis gimbal joysticks (Throttle/Yaw on left, Pitch/Roll on right)
 *  - Toggle switch for Kill/Arm (active LOW = Kill)
 *  - Button 1 (calibration command)
 *  - Button 2 (motor smooth spool command)
 *  - NRF24L01+ wireless module (CE -> D9, CSN -> D10, SPI on D11/D12/D13)
 *  - Nokia 5110 LCD (PCD8544) using hardware SPI (DC -> D5, CS -> D6, RST -> D7)
 *  - Status LED (D4)
 *
 * Features:
 *  - Stick calibration with EEPROM persistence
 *  - Smoothed and deadbanded channel outputs (1000-2000us)
 *  - Guided setup workflow mirrored with the flight controller
 *  - Live telemetry display (throttle, attitude, altitude, status flags)
 *  - NRF24 link watchdog with LED indicator
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// -------------------- Pin Mapping --------------------
static const uint8_t THROTTLE_PIN = A0;
static const uint8_t YAW_PIN = A1;
static const uint8_t PITCH_PIN = A2;
static const uint8_t ROLL_PIN = A3;

static const uint8_t BUTTON1_PIN = 2;
static const uint8_t BUTTON2_PIN = 3;
static const uint8_t STATUS_LED_PIN = 4;
static const uint8_t LCD_DC_PIN = 5;
static const uint8_t LCD_CS_PIN = 6;
static const uint8_t LCD_RST_PIN = 7;
static const uint8_t ARM_TOGGLE_PIN = 8;
static const uint8_t NRF_CE_PIN = 9;
static const uint8_t NRF_CSN_PIN = 10;

static const uint8_t NRF_CHANNEL = 81;
static const byte RADIO_TX_ADDRESS[6] = "FCCTL";
static const byte RADIO_RX_ADDRESS[6] = "TXCTL";

// -------------------- Constants ----------------------
static const uint16_t THROTTLE_MIN = 1000;
static const uint16_t THROTTLE_MAX = 2000;
static const uint16_t SERVO_CENTER = 1500;
static const float STICK_DEADBAND = 0.05f;
static const float STICK_SMOOTH_ALPHA = 0.2f;
static const uint32_t CALIBRATION_MAGIC = 0x52435458; // 'RC TX'
static const uint16_t SEND_INTERVAL_US = 10000; // 100 Hz
static const uint16_t DISPLAY_INTERVAL_MS = 120;
static const uint8_t KILL_ACTIVE_STATE = LOW;

// -------------------- Guide Steps --------------------
enum GuideStep : uint8_t {
  GUIDE_HANDSHAKE = 0,
  GUIDE_KILL_SWITCH_CHECK = 1,
  GUIDE_CALIBRATE_CONFIRM = 2,
  GUIDE_ARM_REQUEST = 3,
  GUIDE_SPOOL_REQUEST = 4,
  GUIDE_READY = 5
};

// -------------------- Radio Structures ---------------
struct RcCommand {
  uint16_t throttle;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  uint8_t switches;
  uint8_t buttons;
  uint8_t guideStep;
  uint8_t reserved;
  uint32_t sequence;
} __attribute__((packed));

struct TelemetryPacket {
  uint16_t throttleEcho;
  int16_t rollDegX10;
  int16_t pitchDegX10;
  int16_t yawDegX10;
  float estimatedAltitudeMeters;
  uint8_t statusFlags;
  uint8_t guideStepAck;
  uint16_t loopMicros;
  uint16_t reserved;
} __attribute__((packed));

// -------------------- Calibration --------------------
struct StickCalibration {
  uint16_t min;
  uint16_t center;
  uint16_t max;
};

struct CalibrationBlock {
  uint32_t magic;
  StickCalibration throttle;
  StickCalibration yaw;
  StickCalibration pitch;
  StickCalibration roll;
};

// -------------------- Globals ------------------------
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
Adafruit_PCD8544 display(LCD_DC_PIN, LCD_CS_PIN, LCD_RST_PIN);

CalibrationBlock calData;
RcCommand command = {};
TelemetryPacket telemetry = {};

GuideStep fcGuideStep = GUIDE_HANDSHAKE;
uint32_t sequenceCounter = 0;

float throttleFiltered = THROTTLE_MIN;
float yawFiltered = SERVO_CENTER;
float pitchFiltered = SERVO_CENTER;
float rollFiltered = SERVO_CENTER;

bool button1State = false;
bool button2State = false;
bool prevButton1State = false;
bool prevButton2State = false;
bool killSwitchActive = true;

bool linkActive = false;
bool telemetryValid = false;

bool spoolRequested = false;
bool inStickCalibration = false;

unsigned long lastAckTimeMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastSendUs = 0;
unsigned long ledFlashUntilMs = 0;

// -------------------- Forward Declarations -----------
void setupRadio();
void loadCalibration();
void saveCalibration();
void ensureCalibrationSanity(StickCalibration &cal);
void maybeRunStickCalibration();
void runStickCalibration();
uint16_t mapThrottleValue(int raw);
uint16_t mapAxisValue(int raw, const StickCalibration &cal);
float applyDeadband(float value, float deadband);
void readInputs();
GuideStep determineGuideCommand(bool spoolButtonPressed, bool spoolButtonReleased);
bool transmitCommand();
void updateDisplay();
const char* stepName(GuideStep step);
void drawGuideMessage(GuideStep step);
void updateStatusLed(bool ackSuccess, bool buttonEvent);

// -------------------- Setup --------------------------
void setup() {
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(ARM_TOGGLE_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  display.begin();
  display.setContrast(57);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0, 0);
  display.println(F("Quad TX Boot"));
  display.println(F("Hold Btns for"));
  display.println(F("Stick Calibrate"));
  display.display();

  Serial.begin(115200);
  Serial.println(F("RC Transmitter booting..."));

  loadCalibration();
  setupRadio();

  delay(500);
  display.clearDisplay();
  display.display();
}

// -------------------- Main Loop ----------------------
void loop() {
  maybeRunStickCalibration();

  if (!inStickCalibration) {
    readInputs();

    const unsigned long nowUs = micros();
    if (nowUs - lastSendUs >= SEND_INTERVAL_US) {
      bool ack = transmitCommand();
      updateStatusLed(ack, (button1State != prevButton1State) || (button2State != prevButton2State));
      prevButton1State = button1State;
      prevButton2State = button2State;
      lastSendUs = nowUs;
    }
  }

  const unsigned long nowMs = millis();
  if (nowMs - lastDisplayMs >= DISPLAY_INTERVAL_MS) {
    updateDisplay();
    lastDisplayMs = nowMs;
  }
}

// -------------------- Radio Setup --------------------
void setupRadio() {
  if (!radio.begin()) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("RF24 Error"));
    display.display();
    while (true) {
      digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
      delay(200);
    }
  }
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(NRF_CHANNEL);
  radio.setCRCLength(RF24_CRC_16);
  radio.setRetries(5, 15);
  radio.enableAckPayload();
  radio.openWritingPipe(RADIO_TX_ADDRESS);
  radio.openReadingPipe(1, RADIO_RX_ADDRESS);
  radio.stopListening();
  Serial.println(F("RF24 ready (TX)"));
}

// -------------------- Calibration Storage ------------
void loadCalibration() {
  EEPROM.get(0, calData);
  if (calData.magic != CALIBRATION_MAGIC) {
    Serial.println(F("No calibration found, loading defaults"));
    calData.magic = CALIBRATION_MAGIC;
    calData.throttle = {200, 512, 820};
    calData.yaw = {200, 512, 820};
    calData.pitch = {200, 512, 820};
    calData.roll = {200, 512, 820};
    saveCalibration();
  }
  ensureCalibrationSanity(calData.throttle);
  ensureCalibrationSanity(calData.yaw);
  ensureCalibrationSanity(calData.pitch);
  ensureCalibrationSanity(calData.roll);
}

void saveCalibration() {
  EEPROM.put(0, calData);
#if defined(ESP8266) || defined(ESP32)
  EEPROM.commit();
#endif
  Serial.println(F("Calibration saved"));
}

void ensureCalibrationSanity(StickCalibration &cal) {
  if (cal.min > cal.center) cal.min = cal.center - 50;
  if (cal.max < cal.center) cal.max = cal.center + 50;
  if (cal.min < 50) cal.min = 50;
  if (cal.max > 970) cal.max = 970;
  if (cal.center < cal.min + 10) cal.center = cal.min + 10;
  if (cal.center > cal.max - 10) cal.center = cal.max - 10;
}

// -------------------- Stick Calibration --------------
void maybeRunStickCalibration() {
  static unsigned long startHoldMs = 0;
  const bool btnCombo = (digitalRead(BUTTON1_PIN) == LOW) && (digitalRead(BUTTON2_PIN) == LOW);
  const bool killHeld = (digitalRead(ARM_TOGGLE_PIN) == KILL_ACTIVE_STATE);
  if (btnCombo && killHeld && !inStickCalibration) {
    if (startHoldMs == 0) {
      startHoldMs = millis();
    } else if (millis() - startHoldMs > 2000) {
      runStickCalibration();
      startHoldMs = 0;
    }
  } else {
    startHoldMs = 0;
  }
}

void runStickCalibration() {
  inStickCalibration = true;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Stick Cal Mode"));
  display.println(F("Center sticks"));
  display.display();

  delay(1500);

  const uint16_t sampleCount = 400;
  uint32_t centerSum[4] = {0, 0, 0, 0};
  for (uint16_t i = 0; i < sampleCount; ++i) {
    centerSum[0] += analogRead(THROTTLE_PIN);
    centerSum[1] += analogRead(YAW_PIN);
    centerSum[2] += analogRead(PITCH_PIN);
    centerSum[3] += analogRead(ROLL_PIN);
    delay(2);
  }

  StickCalibration newThrottle = calData.throttle;
  StickCalibration newYaw = calData.yaw;
  StickCalibration newPitch = calData.pitch;
  StickCalibration newRoll = calData.roll;

  newThrottle.center = centerSum[0] / sampleCount;
  newYaw.center = centerSum[1] / sampleCount;
  newPitch.center = centerSum[2] / sampleCount;
  newRoll.center = centerSum[3] / sampleCount;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Move sticks to"));
  display.println(F("all corners!"));
  display.display();

  uint16_t minVal[4] = {1023, 1023, 1023, 1023};
  uint16_t maxVal[4] = {0, 0, 0, 0};
  const unsigned long sweepDuration = 6000;
  unsigned long start = millis();
  while (millis() - start < sweepDuration) {
    uint16_t reading[4] = {
      static_cast<uint16_t>(analogRead(THROTTLE_PIN)),
      static_cast<uint16_t>(analogRead(YAW_PIN)),
      static_cast<uint16_t>(analogRead(PITCH_PIN)),
      static_cast<uint16_t>(analogRead(ROLL_PIN))
    };
    for (uint8_t i = 0; i < 4; ++i) {
      if (reading[i] < minVal[i]) minVal[i] = reading[i];
      if (reading[i] > maxVal[i]) maxVal[i] = reading[i];
    }
    delay(5);
  }

  newThrottle.min = minVal[0];
  newThrottle.max = maxVal[0];
  newYaw.min = minVal[1];
  newYaw.max = maxVal[1];
  newPitch.min = minVal[2];
  newPitch.max = maxVal[2];
  newRoll.min = minVal[3];
  newRoll.max = maxVal[3];

  ensureCalibrationSanity(newThrottle);
  ensureCalibrationSanity(newYaw);
  ensureCalibrationSanity(newPitch);
  ensureCalibrationSanity(newRoll);

  calData.throttle = newThrottle;
  calData.yaw = newYaw;
  calData.pitch = newPitch;
  calData.roll = newRoll;
  saveCalibration();

  throttleFiltered = mapThrottleValue(analogRead(THROTTLE_PIN));
  yawFiltered = mapAxisValue(analogRead(YAW_PIN), calData.yaw);
  pitchFiltered = mapAxisValue(analogRead(PITCH_PIN), calData.pitch);
  rollFiltered = mapAxisValue(analogRead(ROLL_PIN), calData.roll);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Cal Done"));
  display.println(F("Release sticks"));
  display.display();
  delay(1200);

  inStickCalibration = false;
}

// -------------------- Input Processing ---------------
float applyDeadband(float value, float deadband) {
  if (fabs(value) < deadband) {
    return 0.0f;
  }
  if (value > 0.0f) {
    return (value - deadband) / (1.0f - deadband);
  }
  return (value + deadband) / (1.0f - deadband);
}

uint16_t mapThrottleValue(int raw) {
  const StickCalibration &cal = calData.throttle;
  float norm = (float)(raw - cal.min) / (float)(cal.max - cal.min);
  if (norm < 0.0f) norm = 0.0f;
  if (norm > 1.0f) norm = 1.0f;
  float pwm = THROTTLE_MIN + norm * (THROTTLE_MAX - THROTTLE_MIN);
  if (pwm < THROTTLE_MIN) pwm = THROTTLE_MIN;
  if (pwm > THROTTLE_MAX) pwm = THROTTLE_MAX;
  return static_cast<uint16_t>(pwm);
}

uint16_t mapAxisValue(int raw, const StickCalibration &cal) {
  float spanHigh = (float)(cal.max - cal.center);
  float spanLow = (float)(cal.center - cal.min);
  if (spanHigh < 1.0f) spanHigh = 1.0f;
  if (spanLow < 1.0f) spanLow = 1.0f;

  float norm = 0.0f;
  if (raw >= cal.center) {
    norm = (float)(raw - cal.center) / spanHigh;
  } else {
    norm = (float)(raw - cal.center) / spanLow;
  }
  if (norm > 1.0f) norm = 1.0f;
  if (norm < -1.0f) norm = -1.0f;
  norm = applyDeadband(norm, STICK_DEADBAND);

  float pwm = SERVO_CENTER + norm * 500.0f;
  if (pwm < THROTTLE_MIN) pwm = THROTTLE_MIN;
  if (pwm > THROTTLE_MAX) pwm = THROTTLE_MAX;
  return static_cast<uint16_t>(pwm);
}

void readInputs() {
  int rawThrottle = analogRead(THROTTLE_PIN);
  int rawYaw = analogRead(YAW_PIN);
  int rawPitch = analogRead(PITCH_PIN);
  int rawRoll = analogRead(ROLL_PIN);

  uint16_t targetThrottle = mapThrottleValue(rawThrottle);
  uint16_t targetYaw = mapAxisValue(rawYaw, calData.yaw);
  uint16_t targetPitch = mapAxisValue(rawPitch, calData.pitch);
  uint16_t targetRoll = mapAxisValue(rawRoll, calData.roll);

  throttleFiltered += (targetThrottle - throttleFiltered) * STICK_SMOOTH_ALPHA;
  yawFiltered += (targetYaw - yawFiltered) * STICK_SMOOTH_ALPHA;
  pitchFiltered += (targetPitch - pitchFiltered) * STICK_SMOOTH_ALPHA;
  rollFiltered += (targetRoll - rollFiltered) * STICK_SMOOTH_ALPHA;

  button1State = (digitalRead(BUTTON1_PIN) == LOW);
  button2State = (digitalRead(BUTTON2_PIN) == LOW);
  killSwitchActive = (digitalRead(ARM_TOGGLE_PIN) == KILL_ACTIVE_STATE);
}

// -------------------- Guide Logic --------------------
GuideStep determineGuideCommand(bool spoolPressed, bool spoolReleased) {
  if (!linkActive) {
    spoolRequested = false;
    return GUIDE_HANDSHAKE;
  }

  if (fcGuideStep == GUIDE_SPOOL_REQUEST) {
    if (spoolPressed) {
      spoolRequested = true;
    }
    if (spoolRequested && spoolReleased) {
      return GUIDE_READY;
    }
  }

  if (fcGuideStep == GUIDE_READY) {
    spoolRequested = false;
    return GUIDE_READY;
  }

  return fcGuideStep;
}

// -------------------- Radio Transmission -------------
bool transmitCommand() {
  GuideStep desiredStep = determineGuideCommand(
    button2State && !prevButton2State,
    !button2State && prevButton2State
  );

  command.throttle = killSwitchActive ? THROTTLE_MIN : static_cast<uint16_t>(throttleFiltered);
  command.yaw = static_cast<uint16_t>(yawFiltered);
  command.pitch = static_cast<uint16_t>(pitchFiltered);
  command.roll = static_cast<uint16_t>(rollFiltered);
  command.switches = 0;
  if (killSwitchActive) command.switches |= (1 << 0);
  command.buttons = 0;
  if (button1State) command.buttons |= (1 << 0);
  if (button2State) command.buttons |= (1 << 1);
  command.guideStep = static_cast<uint8_t>(desiredStep);
  command.sequence = sequenceCounter++;

  radio.stopListening();
  bool txOk = radio.write(&command, sizeof(command));
  radio.startListening();

  bool ackSuccess = false;
  if (txOk) {
    while (radio.isAckPayloadAvailable()) {
      TelemetryPacket incoming;
      radio.read(&incoming, sizeof(incoming));
      telemetry = incoming;
      telemetryValid = true;
      fcGuideStep = static_cast<GuideStep>(telemetry.guideStepAck);
      ackSuccess = true;
    }
  }

  if (ackSuccess) {
    linkActive = true;
    lastAckTimeMs = millis();
  } else {
    const unsigned long nowMs = millis();
    if (nowMs - lastAckTimeMs > 300) {
      linkActive = false;
    }
  }

  return ackSuccess;
}

// -------------------- Status LED ---------------------
void updateStatusLed(bool ackSuccess, bool buttonEvent) {
  (void)ackSuccess;
  const unsigned long now = millis();
  if (buttonEvent) {
    ledFlashUntilMs = now + 150;
  }

  bool ledState = false;
  if (linkActive) {
    ledState = ((now / 250) % 2) == 0;
  }
  if (now < ledFlashUntilMs) {
    ledState = true;
  }
  if (!linkActive) {
    ledState = false;
  }
  digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
}

// -------------------- Display Rendering --------------
const char* stepName(GuideStep step) {
  switch (step) {
    case GUIDE_HANDSHAKE: return "Handshake";
    case GUIDE_KILL_SWITCH_CHECK: return "KillCheck";
    case GUIDE_CALIBRATE_CONFIRM: return "Calibrate";
    case GUIDE_ARM_REQUEST: return "Arm";
    case GUIDE_SPOOL_REQUEST: return "Spool";
    case GUIDE_READY: return "Ready";
    default: return "Unknown";
  }
}

void drawGuideMessage(GuideStep step) {
  switch (step) {
    case GUIDE_HANDSHAKE:
      display.print(F("Kill->KILL & wait"));
      break;
    case GUIDE_KILL_SWITCH_CHECK:
      display.print(F("Kill->ARM position"));
      break;
    case GUIDE_CALIBRATE_CONFIRM:
      display.print(F("Press Btn1 Cal"));
      break;
    case GUIDE_ARM_REQUEST:
      display.print(F("Kill=OFF, Thr Low"));
      break;
    case GUIDE_SPOOL_REQUEST:
      display.print(F("Press Btn2 Spin"));
      break;
    case GUIDE_READY:
      display.print(F("Fly: sticks live"));
      break;
    default:
      display.print(F("Step..."));
      break;
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);

  if (inStickCalibration) {
    display.println(F("Calibrating..."));
    display.display();
    return;
  }

  display.print(F("Link:"));
  display.print(linkActive ? F("OK ") : F("-- "));
  display.print(F("Ch"));
  display.print(NRF_CHANNEL);
  display.setCursor(0, 8);
  display.print(F("Step:"));
  display.print(stepName(fcGuideStep));

  display.setCursor(0, 16);
  drawGuideMessage(fcGuideStep);

  display.setCursor(0, 24);
  display.print(F("Thr:"));
  display.print(command.throttle);
  display.print(F(" Y:"));
  display.print(command.yaw);

  display.setCursor(0, 32);
  display.print(F("P:"));
  display.print(command.pitch);
  display.print(F(" R:"));
  display.print(command.roll);

  display.setCursor(0, 40);
  if (telemetryValid) {
    display.print(F("Alt:"));
    display.print(telemetry.estimatedAltitudeMeters, 1);
    display.print(F("m F:"));
    display.print((telemetry.statusFlags & (1 << 1)) ? F("A") : F("S"));
    display.print(F(" K:"));
    display.print(killSwitchActive ? F("K") : F("R"));
  } else {
    display.print(F("Await FC data"));
  }

  display.display();
}
