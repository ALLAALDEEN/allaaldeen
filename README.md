# Arduino Nano Quadcopter Flight Controller & RC Transmitter

This project provides a complete, hobby-grade control stack for a brushed/brushless quadcopter built around two Arduino Nano boards and nRF24L01+ radios:

- **Flight controller**: integrates an MPU6050 IMU, drives 4 ESCs, manages arming logic, and exposes telemetry over the radio link.
- **Transmitter (RC)**: reads dual-axis joysticks, buttons, and a kill toggle, drives a Nokia 5110 LCD for user guidance, and sends calibrated control commands to the quad.

Both sketches are ready to compile with the Arduino IDE and focus on safety-first operation, guided commissioning, and easily repeatable calibration procedures.

---

## Bill of Materials

| Subsystem | Quantity | Component |
|-----------|----------|-----------|
| Flight controller | 1 | Arduino Nano (ATmega328P) |
|  | 1 | MPU6050 6-DoF IMU |
|  | 1 | nRF24L01+ PA/LNA module (with 3.3 V regulator + decoupling) |
|  | 4 | ESCs compatible with PWM (1000–2000 µs) |
|  | 4 | Brushless motors + props |
|  | 1 | Active buzzer (5 V) |
|  | 1 | Status LED + 220 Ω resistor |
|  | misc | Power distribution board, LiPo, frame, wiring |
| RC transmitter | 1 | Arduino Nano (ATmega328P) |
|  | 1 | nRF24L01+ |
|  | 2 | Dual-axis joysticks (gimbals) |
|  | 1 | Two-position toggle switch (Kill/Arm) |
|  | 2 | Momentary push buttons |
|  | 1 | Nokia 5110 LCD (PCD8544) |
|  | 1 | Status LED + 220 Ω resistor |
|  | misc | Enclosure, 9 V or 2S LiPo + regulator |

> ⚠️ **Safety first:** Always remove propellers during bench testing, calibration, or firmware changes.

---

## Pin Mapping

### Flight Controller (`flight_controller/flight_controller.ino`)

| Signal | Nano Pin | Notes |
|--------|----------|-------|
| ESC 1 (front-left) | D3 | Servo PWM (1000–2000 µs) |
| ESC 2 (front-right) | D5 | Servo PWM |
| ESC 3 (rear-right) | D6 | Servo PWM |
| ESC 4 (rear-left) | D9 | Servo PWM |
| nRF24L01 CE | D7 | Radio CE |
| nRF24L01 CSN | D8 | Radio CSN |
| nRF24L01 SCK/MISO/MOSI | D13/D12/D11 | Shared SPI bus |
| MPU6050 SDA/SCL | A4/A5 | I²C bus (enable internal pull-ups) |
| Buzzer | D4 | Uses `tone()` for short beeps |
| Status LED | D13 | Blinks on received packets |
| Battery / ESC power | VIN / 5 V rail | Ensure common ground |

### RC Transmitter (`rc_transmitter/rc_transmitter.ino`)

| Signal | Nano Pin | Notes |
|--------|----------|-------|
| Left stick (Throttle) | A0 | 0–1023 analog |
| Left stick (Yaw) | A1 | 0–1023 analog |
| Right stick (Pitch) | A2 | 0–1023 analog |
| Right stick (Roll) | A3 | 0–1023 analog |
| Kill/Arm toggle | D8 (INPUT_PULLUP) | LOW = Kill engaged |
| Button 1 (Calibrate) | D2 (INPUT_PULLUP) | LOW when pressed |
| Button 2 (Spool) | D3 (INPUT_PULLUP) | LOW when pressed |
| Status LED | D4 | Blinks on healthy link / button events |
| Nokia 5110 DC / CS / RST | D5 / D6 / D7 | Uses hardware SPI (D11/D13) |
| nRF24L01 CE / CSN | D9 / D10 | Shared SPI bus |
| nRF24L01 SCK/MISO/MOSI | D13/D12/D11 | Shared with LCD |

Provide each nRF module with a stable, filtered 3.3 V supply (≥250 mA) and a 10 µF + 0.1 µF decoupling cap placed close to VCC/GND.

---

## Required Arduino Libraries

Flight controller:
- `Servo` (bundled with Arduino IDE)
- `Wire` (bundled)
- `RF24` (by TMRh20)

Transmitter:
- `RF24` (by TMRh20)
- `Adafruit_GFX`
- `Adafruit_PCD8544`

Install external libraries through **Sketch → Include Library → Manage Libraries…** in the Arduino IDE. Select the **Arduino Nano / ATmega328P** board profile for both sketches.

---

## Firmware Deployment

1. Clone or copy the repository.
2. Open `flight_controller/flight_controller.ino` in the Arduino IDE, select the correct serial port, and upload.
3. Open `rc_transmitter/rc_transmitter.ino`, select the second Nano’s port, and upload.
4. Power cycles after upload reset stored EEPROM calibration signatures automatically.

---

## RC Workflow & User Guidance

The transmitter LCD walks you through the exact sequence expected by the flight controller. Each step also aligns with the on-screen prompts and the FC’s buzzer feedback.

| Step | Purpose | LCD Prompt | Action |
|------|---------|------------|--------|
| `GUIDE_HANDSHAKE` | Confirm radio link with FC and kill switch safety | “Kill→KILL & wait” | Power FC, ensure kill toggle is in **Kill** (active LOW) position |
| `GUIDE_KILL_SWITCH_CHECK` | Verify that the toggle can exit Kill | “Kill→ARM position” | Flip the toggle to **Arm** (HIGH) |
| `GUIDE_CALIBRATE_CONFIRM` | Trigger IMU & ESC calibration | “Press Btn1 Cal” | Press & release **Button 1** (remove props before doing this!) |
| `GUIDE_ARM_REQUEST` | Prepare to arm motors | “Kill=OFF, Thr Low” | Leave toggle in Arm, ensure throttle < 1100 µs |
| `GUIDE_SPOOL_REQUEST` | Smooth idle spool demonstration | “Press Btn2 Spin” | Press & hold **Button 2** to start a gentle ramp to 1150 µs |
| `GUIDE_READY` | Full manual control | “Fly: sticks live” | Release button 2, continue into flight once comfortable |

Buzzer feedback (on FC):
- **Short high chirp**: link established.
- **Medium low buzz**: link lost / failsafe.
- **Two short chirps**: calibration completed successfully.

Status LED behavior:
- FC LED (D13): toggles on each received command.
- RC LED (D4): blinks at 2 Hz while link is healthy, latches ON briefly after each button press.

---

## Calibration Procedures

### Stick Calibration (RC)

1. Set the kill toggle to **Kill** and hold both buttons for 2 seconds.
2. On-screen prompts will:
   - Capture joystick centers (leave sticks centered).
   - Instruct you to sweep all axes to their extremes.
3. Calibration data writes to EEPROM and immediately updates filtered channel outputs.

This guarantees mid-stick ≈ 1500 µs, eliminating the “center = 1000 µs” risk described in the requirements.

### IMU & ESC Calibration (FC)

Triggered remotely via **Button 1** when the workflow reaches `GUIDE_CALIBRATE_CONFIRM`.

- IMU: collects 2000 samples over ≈4 s; drone must remain absolutely still on a level surface.
- ESC: outputs 2000 µs for 2 s, then 1000 µs for 3 s (remove propellers). This calibrates throttle range on most hobby ESCs.
- Calibration values store in EEPROM (signature `0xA5A5BEEF`) and auto-load on boot.

During calibration the status flag `isCalibrating` keeps motors cut and telemetry bit 2 asserts (visible on the LCD “F:” field = `S` but calibrating icon available if needed).

---

## Control & Mixing

Flight controller loop runs at 250 Hz:
- Complementary filter fuses gyro/accel for roll & pitch, integrates yaw rate.
- PID gains (default):
  - Roll/Pitch: Kp = 3.5, Ki = 0.02, Kd = 0.18
  - Yaw: Kp = 2.0, Ki = 0.01, Kd = 0.0
- Mixer assumes an **X frame**:
  - Motor 1 front-left
  - Motor 2 front-right
  - Motor 3 rear-right
  - Motor 4 rear-left

Tune gains according to your airframe and prop/ESC response. Start with low Ki, adjust Kp for crisp response, then introduce Kd to damp overshoot.

---

## Telemetry & Display

The FC embeds telemetry in the RF24 ACK payload:

| Field | Description | Display usage |
|-------|-------------|---------------|
| `throttleEcho` | Last received throttle (µs) | LCD line shows `Thr:` |
| `rollDegX10`, `pitchDegX10`, `yawDegX10` | Attitude ×10 degrees | Not presently rendered but available |
| `estimatedAltitudeMeters` | Approximate relative altitude (from accel integration) | LCD bottom line `Alt:` |
| `statusFlags` | Bit0=link, Bit1=armed, Bit2=calibrating, Bit3=spool, Bit4=calibrationValid | LCD field “F:A/S” + potential expansions |
| `guideStepAck` | Current FC workflow step | LCD `Step:` |
| `loopMicros` | Control loop duration | For diagnostics / tuning |

> Altitude estimation using only an MPU6050 accelerometer is prone to drift; treat it as a qualitative indicator. For sustained altitude hold, integrate a barometer (e.g., BMP280) or optical/VL53L0X sensor and extend the telemetry struct.

---

## Safety & Failsafes

- **Radio watchdog:** FC expects fresh packets every ≤500 ms; otherwise it reverts to DISARMED and buzzes low.
- **Kill switch override:** Any time the toggle is in Kill (LOW), FC resets to DISARMED, zeroes integrators, and clamps ESC outputs to 1000 µs.
- **Spool demo:** Button 2 ramps all motors smoothly to 1150 µs minimum spin for motor verification without full flight command authority.
- **Throttle floor:** RC never transmits <1000 µs even with stick jitter; kill switch still forces 1000 µs.
- **EEPROM signatures:** Prevents stale calibration data; if corrupted the firmware falls back to defaults and demands recalibration.

Always test with props removed, battery disconnected from ESCs when flashing, and in an open area for first hover tests.

---

## Extending the System

- **Battery voltage telemetry:** Add a resistor divider to an analog pin on the FC and include it in the telemetry packet.
- **Altitude hold:** Integrate a barometric sensor and upgrade the control loop with a vertical PID.
- **Failsafe land:** Mix throttle ramp-down logic when link drops rather than instant cut (for larger airframes).
- **Logging:** Use an SD card module or external telemetry to log attitude/PID values for tuning.

---

## Repository Layout

```
flight_controller/
  flight_controller.ino   # FC firmware
rc_transmitter/
  rc_transmitter.ino      # RC firmware
README.md                 # System documentation (this file)
```

---

## Quick Start Checklist

- [ ] Flash both Arduino Nano boards with their respective sketches.
- [ ] Power the RC transmitter, perform stick calibration (hold both buttons with kill engaged).
- [ ] Power the quad (without props), follow LCD guidance:
  1. Kill switch to Kill ➜ Wait for link beep.
  2. Kill switch to Arm ➜ LCD advances.
  3. Press Button 1 ➜ IMU + ESC calibration (props still off).
  4. Kill to Arm, throttle low ➜ Request arm.
  5. Press Button 2 ➜ Observe smooth spin-up (still props off).
- [ ] Install props, repeat the workflow, and perform a cautious hover test.

You now have a stable, guided quadcopter platform with explicit safety interlocks and clear user feedback channels. Happy flying! 🛸

