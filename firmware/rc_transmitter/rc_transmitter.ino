#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// ----------------------------- Pin Map ---------------------------------------

// NRF24L01 (CE, CSN)
static RF24 radio(7, 8);
static const uint64_t RADIO_PIPE = 0xE8E8F0F0E1LL;

// Nokia 5110 LCD
static const uint8_t LCD_DC_PIN = 4;
static const uint8_t LCD_CE_PIN = 6;
static const uint8_t LCD_RST_PIN = 5;
static const uint8_t LCD_DIN_PIN = 11;  // MOSI
static const uint8_t LCD_CLK_PIN = 13;  // SCK
static Adafruit_PCD8544 display(LCD_CLK_PIN, LCD_DIN_PIN, LCD_DC_PIN, LCD_CE_PIN, LCD_RST_PIN);

// Controls
static const uint8_t TOGGLE_PIN = 2;
static const uint8_t BUTTON_CAL_PIN = 9;
static const uint8_t BUTTON_START_PIN = 3;
static const uint8_t STATUS_LED_PIN = A4;
static const uint8_t BUZZER_PIN = 10;

// Joysticks
static const uint8_t JOY_THROTTLE_PIN = A0;
static const uint8_t JOY_YAW_PIN = A1;
static const uint8_t JOY_PITCH_PIN = A2;
static const uint8_t JOY_ROLL_PIN = A3;

// ----------------------------- Data structures --------------------------------

enum RadioFlags : uint8_t {
  FLAG_ARM_REQUEST     = 0x01,
  FLAG_KILL_SWITCH     = 0x02,
  FLAG_CALIBRATE       = 0x04,
  FLAG_MOTOR_TEST      = 0x08,
  FLAG_ACK             = 0x10
};

struct __attribute__((packed)) RcPacket {
  uint8_t header;     // 0xA5
  uint8_t seq;
  uint8_t flags;
  uint16_t throttle;  // 1000-2000 us
  int16_t roll;       // centideg
  int16_t pitch;      // centideg
  int16_t yaw;        // centideg/sec
  uint16_t aux;       // button bitfield
  uint8_t crc;
};

struct __attribute__((packed)) FcPacket {
  uint8_t header;     // 0x5A
  uint8_t seq;
  uint8_t status;
  uint16_t batteryMv;
  int16_t roll;
  int16_t pitch;
  int16_t yawRate;
  uint16_t throttle;
  int16_t altitude;
  uint8_t crc;
};

enum FcStatusFlags : uint8_t {
  STATUS_ARMED     = 0x01,
  STATUS_CALIB     = 0x02,
  STATUS_FAILSAFE  = 0x04,
  STATUS_IMU_OK    = 0x08,
  STATUS_MOTORS    = 0x10
};

// Joystick calibration
struct AxisCal {
  int16_t minVal;
  int16_t centerVal;
  int16_t maxVal;
};

struct CalibrationBlock {
  uint32_t magic;
  AxisCal throttle;
  AxisCal yaw;
  AxisCal pitch;
  AxisCal roll;
};

static const uint32_t CAL_MAGIC = 0xA55A4352UL;  // 'RCSA'
static CalibrationBlock calData;

// ----------------------------- UI State Machine -------------------------------

enum UiState {
  UI_WAIT_LINK,
  UI_CHECK_KILL,
  UI_REQUEST_CAL,
  UI_CALIBRATING,
  UI_ARM_PROMPT,
  UI_SOFTSTART_PROMPT,
  UI_READY,
  UI_FAILSAFE
};

static UiState uiState = UI_WAIT_LINK;

// ----------------------------- Globals ----------------------------------------

static RcPacket txPacket = {};
static FcPacket rxPacket = {};
static bool fcLinked = false;
static unsigned long lastRxMillis = 0;
static uint8_t seqCounter = 0;
static bool calibrationRequested = false;
static unsigned long calibHoldTimer = 0;
static const unsigned long CALIB_HOLD_MS = 200;  // hold request for 200ms

static unsigned long lastUiRefresh = 0;
static unsigned long lastInputSample = 0;
// Debounce
static bool prevCalButton = false;
static bool prevStartButton = false;

// Axis smoothing
static float smoothedThrottle = 0.0f;
static float smoothedYaw = 0.0f;
static float smoothedPitch = 0.0f;
static float smoothedRoll = 0.0f;

// ----------------------------- Utility ----------------------------------------

static uint8_t crc8_dallas(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    uint8_t inbyte = *data++;
    for (uint8_t i = 8; i; --i) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  return crc;
}

static void toneBeep(uint16_t freq, uint16_t durMs) {
  tone(BUZZER_PIN, freq, durMs);
  lastBuzzerTone = millis();
}

static void ensureCalibrationDefaults() {
  if (calData.magic != CAL_MAGIC ||
      calData.throttle.minVal >= calData.throttle.maxVal) {
    calData.magic = CAL_MAGIC;
    calData.throttle = {200, 512, 820};
    calData.yaw = {220, 512, 800};
    calData.pitch = {220, 512, 800};
    calData.roll = {220, 512, 800};
    EEPROM.put(0, calData);
  }
}

static void loadCalibration() {
  EEPROM.get(0, calData);
  ensureCalibrationDefaults();
}

static void saveCalibration() {
  calData.magic = CAL_MAGIC;
  EEPROM.put(0, calData);
}

static float applyDeadband(float value, float deadband) {
  if (fabs(value) < deadband) return 0.0f;
  if (value > 0.0f) return value - deadband;
  return value + deadband;
}

static float mapAxis(int raw, const AxisCal &cal, bool isThrottle) {
  if (isThrottle) {
    float norm = (float)(raw - cal.minVal) / (float)(cal.maxVal - cal.minVal);
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    return norm;
  } else {
    if (raw >= cal.centerVal) {
      float span = (float)(cal.maxVal - cal.centerVal);
      if (span < 1.0f) span = 1.0f;
      float norm = (float)(raw - cal.centerVal) / span;
      if (norm > 1.0f) norm = 1.0f;
      return norm;
    } else {
      float span = (float)(cal.centerVal - cal.minVal);
      if (span < 1.0f) span = 1.0f;
      float norm = (float)(raw - cal.centerVal) / span;
      if (norm < -1.0f) norm = -1.0f;
      return norm;
    }
  }
}

static float lowpass(float prev, float input, float alpha) {
  return prev + alpha * (input - prev);
}

// ----------------------------- Joystick sampling ------------------------------

static void sampleJoysticks() {
  const unsigned long SAMPLE_PERIOD_MS = 5;
  unsigned long now = millis();
  if (now - lastInputSample < SAMPLE_PERIOD_MS) {
    return;
  }
  lastInputSample = now;

  int rawThrottle = analogRead(JOY_THROTTLE_PIN);
  int rawYaw = analogRead(JOY_YAW_PIN);
  int rawPitch = analogRead(JOY_PITCH_PIN);
  int rawRoll = analogRead(JOY_ROLL_PIN);

  float thrNorm = mapAxis(rawThrottle, calData.throttle, true);
  float yawNorm = mapAxis(rawYaw, calData.yaw, false);
  float pitchNorm = mapAxis(rawPitch, calData.pitch, false);
  float rollNorm = mapAxis(rawRoll, calData.roll, false);

  yawNorm = applyDeadband(yawNorm, 0.05f);
  pitchNorm = applyDeadband(pitchNorm, 0.05f);
  rollNorm = applyDeadband(rollNorm, 0.05f);

  smoothedThrottle = lowpass(smoothedThrottle, thrNorm, 0.2f);
  smoothedYaw = lowpass(smoothedYaw, yawNorm, 0.3f);
  smoothedPitch = lowpass(smoothedPitch, pitchNorm, 0.3f);
  smoothedRoll = lowpass(smoothedRoll, rollNorm, 0.3f);
}

// ----------------------------- Calibration routine ----------------------------

static void runJoystickCalibration() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("CAL MODE"));
  display.println(F("Move sticks"));
  display.println(F("to limits."));
  display.display();

  AxisCal newThr = {1023, 512, 0};
  AxisCal newYaw = {1023, 512, 0};
  AxisCal newPitch = {1023, 512, 0};
  AxisCal newRoll = {1023, 512, 0};

  unsigned long start = millis();
  while (millis() - start < 6000UL) {
    int vThr = analogRead(JOY_THROTTLE_PIN);
    int vYaw = analogRead(JOY_YAW_PIN);
    int vPitch = analogRead(JOY_PITCH_PIN);
    int vRoll = analogRead(JOY_ROLL_PIN);

    newThr.minVal = min(newThr.minVal, vThr);
    newThr.maxVal = max(newThr.maxVal, vThr);

    newYaw.minVal = min(newYaw.minVal, vYaw);
    newYaw.maxVal = max(newYaw.maxVal, vYaw);

    newPitch.minVal = min(newPitch.minVal, vPitch);
    newPitch.maxVal = max(newPitch.maxVal, vPitch);

    newRoll.minVal = min(newRoll.minVal, vRoll);
    newRoll.maxVal = max(newRoll.maxVal, vRoll);

    delay(5);
  }

  // Capture center values
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Hold sticks"));
  display.println(F("center..."));
  display.display();
  delay(1500);

  AxisCal newCals[4] = {newThr, newYaw, newPitch, newRoll};
  for (uint8_t i = 0; i < 4; ++i) {
    long sum = 0;
    for (uint8_t s = 0; s < 50; ++s) {
      int reading = analogRead(JOY_THROTTLE_PIN + i);
      sum += reading;
      delay(5);
    }
    newCals[i].centerVal = sum / 50;
  }

  calData.throttle = newCals[0];
  calData.yaw = newCals[1];
  calData.pitch = newCals[2];
  calData.roll = newCals[3];
  saveCalibration();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Calibration"));
  display.println(F("saved."));
  display.display();
  toneBeep(1400, 200);
  delay(800);

  // Reset smoothed values
  smoothedThrottle = 0.0f;
  smoothedYaw = 0.0f;
  smoothedPitch = 0.0f;
  smoothedRoll = 0.0f;
}

// ----------------------------- UI helpers -------------------------------------

static void drawCentered(const __FlashStringHelper *line1,
                         const __FlashStringHelper *line2 = nullptr,
                         const __FlashStringHelper *line3 = nullptr,
                         const __FlashStringHelper *line4 = nullptr) {
  display.clearDisplay();
  display.setCursor(0, 0);
  if (line1) display.println(line1);
  if (line2) display.println(line2);
  if (line3) display.println(line3);
  if (line4) display.println(line4);
  display.display();
}

static void drawTelemetry() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(F("THR "));
  display.print(rxPacket.throttle);
  display.print(F("us"));

  display.setCursor(0, 10);
  display.print(F("Roll "));
  display.print(rxPacket.roll / 100.0f, 1);
  display.setCursor(0, 20);
  display.print(F("Pitch "));
  display.print(rxPacket.pitch / 100.0f, 1);

  display.setCursor(0, 30);
  display.print(F("Yaw "));
  display.print(rxPacket.yawRate / 100.0f, 1);

  display.setCursor(0, 40);
  display.print(F("Alt "));
  display.print(rxPacket.altitude);
  display.print(F("cm"));

  display.setCursor(0, 50);
  display.print(F("Bat "));
  display.print(rxPacket.batteryMv / 1000.0f, 2);
  display.print(F("V"));
  display.display();
}

// ----------------------------- Radio ------------------------------------------

static void sendRadioPacket(uint8_t flags) {
  txPacket.header = 0xA5;
  txPacket.seq = seqCounter++;
  txPacket.flags = flags;
  txPacket.aux = 0;

  // Map throttle to 1000-2000us, keep floor at 1100 in armed state
  float throttleNorm = smoothedThrottle;
  uint16_t throttleUs = 1000 + (uint16_t)(throttleNorm * 1000.0f);
  if (flags & FLAG_KILL_SWITCH) {
    throttleUs = 1000;
  } else {
    if (throttleUs < 1100) throttleUs = 1100;
  }
  txPacket.throttle = throttleUs;

  // Map attitude commands (max 25 deg) and yaw rate (max 150 deg/s)
  txPacket.roll = (int16_t)(smoothedRoll * 2500.0f);
  txPacket.pitch = (int16_t)(smoothedPitch * 2500.0f);
  txPacket.yaw = (int16_t)(smoothedYaw * 15000.0f);

  txPacket.crc = crc8_dallas((uint8_t *)&txPacket, sizeof(txPacket) - 1);

  radio.stopListening();
  radio.write(&txPacket, sizeof(txPacket));
  radio.startListening();
}

static void readTelemetry() {
  while (radio.available()) {
    radio.read(&rxPacket, sizeof(rxPacket));
    if (rxPacket.header != 0x5A) continue;
    uint8_t crc = crc8_dallas((uint8_t *)&rxPacket, sizeof(rxPacket) - 1);
    if (crc != rxPacket.crc) continue;
    fcLinked = true;
    lastRxMillis = millis();
  }

  if (fcLinked && (millis() - lastRxMillis) > 500) {
    fcLinked = false;
    uiState = UI_WAIT_LINK;
  }
}

// ----------------------------- UI logic ---------------------------------------

static void updateUiState(bool killActive, bool armButton, bool calibrBtn) {
  if (!fcLinked) {
    uiState = UI_WAIT_LINK;
    return;
  }

  switch (uiState) {
    case UI_WAIT_LINK:
      uiState = UI_CHECK_KILL;
      toneBeep(2200, 120);
      break;

    case UI_CHECK_KILL:
      if (killActive) {
        uiState = UI_REQUEST_CAL;
      }
      break;

    case UI_REQUEST_CAL:
      if (calibrationRequested) {
        uiState = UI_CALIBRATING;
      }
      break;

    case UI_CALIBRATING:
      if ((rxPacket.status & STATUS_CALIB) && !calibrationRequested) {
        uiState = UI_ARM_PROMPT;
      }
      break;

    case UI_ARM_PROMPT:
      if (!killActive) {
        uiState = UI_SOFTSTART_PROMPT;
      }
      break;

    case UI_SOFTSTART_PROMPT:
      if (armButton) {
        uiState = UI_READY;
      }
      break;

    case UI_READY:
      if (rxPacket.status & STATUS_FAILSAFE) {
        uiState = UI_FAILSAFE;
      }
      break;

    case UI_FAILSAFE:
      if (!(rxPacket.status & STATUS_FAILSAFE) && killActive) {
        uiState = UI_CHECK_KILL;
      }
      break;
  }
}

static void renderUi() {
  unsigned long now = millis();
  if (now - lastUiRefresh < 150) {
    return;
  }
  lastUiRefresh = now;

  switch (uiState) {
    case UI_WAIT_LINK:
      drawCentered(F("Searching"), F("for FC..."));
      break;
    case UI_CHECK_KILL:
      drawCentered(F("Set toggle"), F("to KILL"), F("before setup"));
      break;
    case UI_REQUEST_CAL:
      drawCentered(F("Press BTN1"), F("to start"), F("calibration"));
      break;
    case UI_CALIBRATING:
      drawCentered(F("Calibrating"), F("IMU & ESC"), F("Hold still..."));
      break;
    case UI_ARM_PROMPT:
      drawCentered(F("Flip toggle"), F("to ARM mode"), F("Throttle min"));
      break;
    case UI_SOFTSTART_PROMPT:
      drawCentered(F("Press BTN2"), F("for motor"), F("soft start"));
      break;
    case UI_READY:
      drawTelemetry();
      break;
    case UI_FAILSAFE:
      drawCentered(F("FAILSAFE!"), F("Toggle to KILL"), F("Check link"));
      break;
  }
}

// ----------------------------- Setup ------------------------------------------

void setup() {
  pinMode(TOGGLE_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CAL_PIN, INPUT_PULLUP);
  pinMode(BUTTON_START_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  display.begin();
  display.setContrast(55);
  display.clearDisplay();
  display.display();

  loadCalibration();

  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(108);
  radio.setRetries(5, 15);
  radio.openWritingPipe(RADIO_PIPE);
  radio.openReadingPipe(1, RADIO_PIPE);
  radio.startListening();

  toneBeep(1800, 120);
}

// ----------------------------- Main loop --------------------------------------

void loop() {
  sampleJoysticks();
  readTelemetry();

  bool killActive = digitalRead(TOGGLE_PIN) == LOW;
  bool calPressed = !digitalRead(BUTTON_CAL_PIN);
  bool startPressed = !digitalRead(BUTTON_START_PIN);

  bool calEdge = calPressed && !prevCalButton;
  bool startEdge = startPressed && !prevStartButton;
  prevCalButton = calPressed;
  prevStartButton = startPressed;

  // Calibration button
  if (calEdge) {
    toneBeep(1600, 100);
    runJoystickCalibration();
    calibrationRequested = true;
    calibHoldTimer = millis();
    uiState = UI_CALIBRATING;
  }

  if (calibrationRequested && (millis() - calibHoldTimer) > CALIB_HOLD_MS) {
    calibrationRequested = false;
  }

  // Determine flags
  uint8_t flags = 0;
  if (!killActive) {
    flags |= FLAG_ARM_REQUEST;
  } else {
    flags |= FLAG_KILL_SWITCH;
  }
  if (calibrationRequested) {
    flags |= FLAG_CALIBRATE;
  }
  if (startEdge) {
    flags |= FLAG_MOTOR_TEST;
  }

  sendRadioPacket(flags);

  updateUiState(killActive, startEdge, calibrationRequested);
  renderUi();

  if (fcLinked) {
    digitalWrite(STATUS_LED_PIN, (rxPacket.status & STATUS_ARMED) ? HIGH : LOW);
  } else {
    digitalWrite(STATUS_LED_PIN, (millis() / 200) % 2);
  }
}
