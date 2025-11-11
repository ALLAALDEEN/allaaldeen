## Operation Guide

### Required Arduino Libraries
- **Flight Controller (`flight_controller.ino`):**
  - `Wire` (Arduino core)
  - `Servo` (Arduino core)
  - `RF24` by TMRh20 (`Tools → Manage Libraries → RF24 by TMRh20`)
  - `I2Cdevlib MPU6050` (`MPU6050.h`). Install from [https://github.com/ElectronicCats/mpu6050](https://github.com/ElectronicCats/mpu6050) or Jeff Rowberg’s I2Cdevlib.
- **Radio Controller (`rc_transmitter.ino`):**
  - `RF24` by TMRh20
  - `Adafruit GFX`
  - `Adafruit PCD8544 Nokia 5110 LCD`

After installing, select **Arduino Nano**, **Processor: ATmega328P (Old Bootloader)** if necessary, and set the correct COM port before uploading.

### Firmware Upload
1. Connect the Arduino Nano via USB.
2. Open the corresponding `.ino` file in the Arduino IDE.
3. Ensure the required libraries are installed (see above).
4. Choose the correct board/processor/port.
5. Click **Upload**.
6. Repeat for the other Nano.

### Pre-Flight Hardware Checklist
- Propellers removed during all calibration and bench testing.
- ESC signal grounds tied to flight controller ground.
- NRF24L01 modules powered from a clean 3.3 V regulator with >47 µF bulk capacitor.
- MPU6050 mounted rigidly with foam isolation and oriented according to firmware assumptions (X forward, Y right, Z up).
- LiPo connected to power distribution board with a 5 V BEC powering both Nanos.
- Toggle switch wired so that `LOW` equals **Kill** (connected to GND), `HIGH` equals **Arm enable**.
- Buttons wired to pull the input low when pressed (firmware uses `INPUT_PULLUP`).

### RC User Interface Flow (Nokia 5110)
The transmitter walks the pilot through the startup sequence with explicit instructions:

1. **Searching for FC** – Displays “Searching for FC…” until the radio link is established. The FC buzzer chirps once when it receives the first valid packet. Status LED on RC blinks.
2. **Set Kill Switch** – Prompt: “Set toggle to KILL before setup.” Ensure the toggle is in the Kill position (LOW). Once detected, the prompt advances.
3. **Press BTN1 to start calibration** – Prompt instructs pressing Button 1. This performs:
   - Joystick auto-calibration (move each stick to limits when prompted, then center).
   - After sticks are calibrated locally, the RC requests FC IMU & ESC calibration (flag held for ≥200 ms). FC buzzer sounds during calibration.
4. **Calibrating IMU & ESC** – Display shows “Calibrating IMU & ESC, Hold still…”. Keep the quad flat and motionless. ESCs emit calibration tones; FC stores offsets in EEPROM.
5. **Flip Toggle to ARM mode** – When FC reports calibration complete, the display instructs moving the toggle to the Arm position. Ensure throttle is at minimum (RC enforces 1100 µs floor when armed).
6. **Press BTN2 for motor soft start** – Initiates FC soft-start (gradually ramps motors to idle over 3 s). Motors spin slowly but drone remains grounded.
7. **Ready to Fly** – Display switches to telemetry dashboard:
   - Line 1: `THR ####us`
   - Lines 2–4: `Roll`, `Pitch`, `Yaw` (attitude & rate feedback)
   - Line 5: `Alt ###cm` (placeholder) and `Bat #.##V`
   - Status LED solid when armed, blinking when disarmed or link lost.
   - Buzzer beeps if failsafe or link loss occurs.
8. **Failsafe** – If radio packets stop or tilt exceeds threshold, FC cuts motors. Display shows “FAILSAFE! Toggle to KILL, Check link”. Return toggle to Kill, resolve issue, then repeat the arming flow.

### Button & Switch Behaviour
- **Toggle Switch (Kill/Arm)**: Must stay in Kill to prevent arming. The RC sends `FLAG_KILL_SWITCH` when LOW, and `FLAG_ARM_REQUEST` when HIGH.
- **Button 1 (Calibration)**: Triggers stick calibration and requests FC IMU/ESC calibration. Hold drone still until FC finishing chirp. Calibration data persists in EEPROM on both devices.
- **Button 2 (Motor Test / Soft Start)**: Sends `FLAG_MOTOR_TEST` to start the FC soft-start state. If held for >2 s while already armed, FC disarms motors (safety override).
- **Joysticks**: RC normalizes stick travel; throttle is mapped 0–100% → 1100–2000 µs (Kill state forces 1000 µs). Roll, pitch commands ±25°; yaw rate ±150°/s. Deadband is applied around center to remove drift.

### Flight Controller Indicators
- **Status LED**: Solid when motors are running (armed), off when disarmed, flashing in failsafe.
- **Buzzer**:
  - 2 short chirps on boot.
  - Single chirp when arming.
  - Continuous rapid chirp if radio link is lost.
  - Long tone when calibration begins/ends.
- **ESC Outputs**: 1000 µs while disarmed, 1120 µs idle when armed, up to 2000 µs during flight.

### Calibration Details
- **IMU Calibration**: Averages 2000 samples to compute gyro and accel biases. Requires the frame to remain motionless on a level surface. Stored in EEPROM (`CalibrationData`).
- **ESC Calibration**: Sends 2000 µs for 3 s then 1000 µs for 3 s to all ESCs when Button 1 is pressed.
- **Joystick Calibration**: Captures min/max/center for each axis over 6 s of movement plus a 1.5 s center capture. Values saved in RC EEPROM to prevent throttle-centered-at-1000 µs issues.

### Safety Workflow Summary
1. Power RC → FC.
2. Wait for link confirmation (buzzer + LCD).
3. Ensure toggle = Kill (display prompt).
4. Press Button 1 → complete both RC and FC calibrations.
5. Flip toggle to Arm, confirm throttle at minimum.
6. Press Button 2 for soft motor start.
7. Increase throttle to take off; monitor telemetry.
8. To land/disarm: lower throttle, flip toggle to Kill (or hold Button 2 for >2 s).
9. If failsafe triggered, motors cut automatically; follow LCD instructions to recover.

### Tuning & Adjustments
- **PID Gains** (FC firmware): adjust `pidRate*` and `pidAngle*` arrays. Increase `pidAngleKp` for stronger attitude hold, adjust `pidRateKd` to reduce oscillations.
- **Complementary Filter**: `COMPLEMENTARY_ALPHA` controls gyro vs accel weight; lower slightly if long-term drift observed, raise if noise high.
- **Throttle Floor**: Change `ESC_IDLE_US` and `ESC_SOFTSTART_TARGET` to match ESC/motor characteristics.
- **Deadband**: Modify `applyDeadband()` parameters for RC axes to preference.
- **Telemetry Rate**: Adjust `lastTelemetryMicros` threshold (currently 25 Hz).

### Troubleshooting
- **No NRF link**:
  - Verify 3.3 V supply and bulk capacitor on NRF24L01.
  - Match RF channel/address on both sketches.
  - Check CE/CSN wiring.
- **Motors twitch on boot**:
  - Confirm ESC signal grounds secure.
  - Ensure throttle calibration completed (LCD prompt).
  - Verify `ESC_IDLE_US` > ESC minimum spin-up pulse.
- **Drifts / unstable hover**:
  - Redo IMU calibration on a level surface.
  - Inspect MPU6050 mounting for vibration isolation.
  - Tweak PID gains; start with small increments.
- **LCD blank or garbled**:
  - Recheck wiring (5 V only for backlight, logic 3.3 V).
  - Ensure `Adafruit_PCD8544` contrast set appropriately.
- **Joystick center not zero**:
  - Run Button 1 calibration again.
  - Replace joystick potentiometers if noisy.

### Next Steps & Expansion
- Integrate a BMP280/BME280 for real altitude feedback (replace placeholder in telemetry).
- Add SD logging via SPI for PID tuning analysis.
- Implement headless mode or altitude hold by extending RC flags and FC state machine.
- Consider carbon fiber frame upgrades and prop balancing for smoother flight.
