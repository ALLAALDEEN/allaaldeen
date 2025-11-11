## Bring-Up & Operation Guide

This guide walks you through flashing the firmware, validating the NRF24L01+ radio link, performing all required calibrations, and flying the quadcopter safely.

### 1. Required Arduino Libraries
Install these libraries via the Arduino Library Manager before compiling either sketch:
- `RF24` by TMRh20
- `Adafruit PCD8544 Nokia 5110 LCD library` and its dependency `Adafruit GFX`
- `I2Cdevlib` MPU6050 driver (Jeff Rowberg) - install the `MPU6050` library

### 2. Flashing Firmware
1. **Flight Controller**: Open `firmware/flight_controller/flight_controller.ino`. Select `Arduino Nano` (ATmega328P 16 MHz). Verify the board uses the *Old Bootloader* option if you have upload issues. Upload to the Nano on the aircraft.
2. **RC Transmitter**: Open `firmware/radio_controller/radio_controller.ino` and upload to the Nano on the transmitter.

After flashing, power both devices with their normal supply (5 V BEC on the aircraft, regulated 5 V inside the transmitter).

### 3. Initial Hardware Checks
- Confirm the MPU6050 is oriented correctly on the airframe; the sketch assumes an X-layout (front motors at ESC pins D3/D5).
- Verify NRF24L01+ modules receive a clean 3.3 V with a 100 uF capacitor placed at the module pins.
- Remove all propellers for bench testing.
- Ensure the Arm/Kill toggle pulls the input to GND when in **Kill**; the sketch enables the internal pull-up.
- Buzzer polarity: positive lead connects to the designated Arduino output through the buzzer module's transistor (or direct for active buzzers).

### 4. RC Joystick Calibration (Fixes the Throttle Center Issue)
The transmitter enforces a calibration wizard on first boot if EEPROM lacks valid data:
1. Power the transmitter while holding the throttle at **minimum**.
2. When prompted "Joystick Cal - Press Btn2", press Button 2 to start.
3. Hold all sticks at their centers (except throttle at minimum) and press Button 2 again to record the centers.
4. Move every axis — including throttle — to all extremes for 8 seconds when prompted.
5. Set the throttle back to minimum and press Button 2. Two short beeps confirm success.

The throttle axis now maps **minimum stick** -> `1000 us`, **maximum stick** -> `2000 us`, so the previous dangerous mid-stick behaviour is eliminated. Deadband and scaling for roll/pitch/yaw are derived from your calibration; repeat this wizard anytime the joysticks are serviced (hold Button 2 during power-on to re-run).

### 5. Link Verification
1. Power both the RC transmitter and the flight controller (props removed).
2. The RC LCD shows "NRF Link - Searching" until it receives telemetry. Once linked, both sides emit a short beep and the status LED on the FC flashes.
3. If the screen stays on "Searching", check wiring, antenna orientation, and ensure both modules use channel 110 (hard-coded).

### 6. Guided Pre-Flight Flow (Display Prompts)
Follow the Nokia 5110 display prompts one by one:
1. **Toggle KILL** - move the Arm/Kill switch into the Kill position (towards GND). The transmitter confirms with two beeps.
2. **Press Btn1 - Calibrate IMU and ESC** - press Button 1. Keep the quad absolutely still. The FC:
   - Averages 2000 IMU samples for gyro/accel offsets.
   - Performs an ESC calibration (max, then min pulses). The buzzer and status LED provide feedback.
   - Saves calibration to EEPROM.
3. When the display reports **Toggle ARM**, flip the switch to Arm (release from ground). The FC checks throttle is at absolute minimum before arming.
4. **Press Btn2 - Motor soft spin**: hold Button 2 to command a gentle ramp to idle (1150 us). Release to exit the test.
5. The display then switches to the live telemetry page showing:
   - Flight controller state (`READY`, `ARMED`, etc.)
   - Throttle percentage
   - Roll / Pitch / Yaw (tenths of a degree)
   - Estimated altitude (metres, relative to arming point)
   - Link quality (age of last ACK in ms)
6. Once you see "ARMED" and motors spinning smoothly, you are cleared to raise the throttle and fly.

### 7. Normal Flying
- Throttle (left stick vertical) controls lift with a linear 1000-2000 us mapping.
- Yaw (left stick horizontal) commands +/-400 deg/s and is stabilised with a rate PID loop.
- Pitch/Roll (right stick) command +/-30 deg attitude targets with deadband to prevent drift.
- The FC mixes outputs for an X-frame: front-left/front-right/rear-right/rear-left on D3/D5/D6/D9 respectively.

### 8. Failsafe Behaviour
- **Kill switch engaged**: FC forces all ESC pulses to 1000 us, regardless of other inputs.
- **Link loss (>200 ms)**: FC drops to failsafe (SAFE_KILL), stops motors, silences the buzzer, and waits for the link to return.
- **Button 2 held** while armed: motors ramp to idle only - lift-off requires manual throttle afterward.
- **Button 1** outside the wizard triggers full IMU+ESC calibration again; repeat anytime sensors drift.

### 9. Safety Reminders
- Complete calibrations and the motor spin test with propellers removed. Install props only after verifying even RPM based on tone and telemetry.
- Mount the MPU6050 on vibration-damping foam and shield it from ESC cables to minimise EMI.
- Use a safety strap or tether during the first hover. Start with conservative PID tuning (defaults supplied) before increasing agility.
- Monitor LiPo voltage externally; `telemetry.batteryMv` is available if you connect a voltage divider to A0 on the FC.

### 10. Troubleshooting Quick Reference
| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| RC stuck on "NRF Link - Searching" | Wiring error, wrong CE/CSN pins, insufficient 3.3 V | Verify wiring, add 100 uF cap, ensure antennas upright |
| Motors won't arm | Kill switch still engaged, throttle not at minimum, calibration flag invalid | Follow the display steps; redo joystick calibration |
| Motors desync at spin test | ESC calibration incomplete, BEC sagging | Re-run calibration, confirm dedicated 5 V BEC |
| Drift in hover | IMU not level during calibration, frame vibration | Re-calibrate on a flat surface, balance props, add foam mount |
| Altitude estimate drifts | Expected (IMU-only). Add a BMP280 or VL53L1X sensor for better altitude hold |

Keep this guide handy during bring-up to ensure a controlled and repeatable workflow from power-up to flight.
