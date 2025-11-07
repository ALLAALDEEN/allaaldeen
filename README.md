## Nano Drone Controller

Custom quadcopter stack built around dual Arduino Nano boards. The flight controller integrates IMU + barometer sensing, NRF24L01 radio reception, PID attitude stabilization, and ESC outputs. The handheld RC transmitter reads dual joysticks, toggle switches, buttons, and potentiometers, packaging them into NRF24 frames with optional telemetry feedback.

### Repository Layout
- `flight_controller/flight_controller.ino` – firmware for the airframe Nano (MPU6050, MS5611, NRF24L01, ESC + buzzer/LED control, PID loop).
- `rc_controller/rc_controller.ino` – firmware for the handheld TX Nano (joysticks, switches, buttons, pots, NRF24L01, telemetry viewer).

### Required Arduino Libraries
Install these via **Sketch → Include Library → Manage Libraries…** on both projects:
- `RF24` by TMRh20
- `MPU6050` by Electronic Cats (I2Cdev-based)
- `MS5611` by Rob Tillaart
- `Servo` (bundled with Arduino IDE)

### Pin Map (as coded)
**Flight Controller (Nano + quadcopter)**
- NRF24L01: `CE → D4`, `CSN → D10`, SPI pins to `D13/D12/D11`, `3.3 V`, `GND`.
- MPU6050: `SDA → A4`, `SCL → A5`, `INT → D2`, `VCC 3.3 V`, `GND`.
- MS5611 (GY-36): shares `SDA/SCL` on `A4/A5`, `VCC 3.3 V`, `GND`.
- ESC signal outputs: `Front-Left D3`, `Front-Right D5`, `Rear-Right D6`, `Rear-Left D7`.
- Buzzer: `D8`.
- Status LED: `D13` (onboard LED; change `PIN_LED` in code if you want a dedicated LED).

**RC Controller (Nano handheld)**
- NRF24L01: `CE → D9`, `CSN → D10`, SPI pins to `D13/D12/D11`, `3.3 V`, `GND`.
- Left joystick: `V → A0` (throttle), `H → A1` (yaw), `SEL → A4` (momentary push).
- Right joystick: `V → A2` (pitch), `H → A3` (roll), `SEL → A5`.
- Toggle switches: `SW1 → D2`, `SW2 → D3` (other terminals → GND).
- Push buttons: `D4–D7` (wired to ground, using internal pull-ups).
- Pots: `A6`, `A7`.

### Uploading the Firmware
1. Open each `.ino` in the Arduino IDE.
2. Select `Arduino Nano`, the correct processor (ATmega328P/Old Bootloader if applicable), and the right serial port.
3. Upload `rc_controller.ino` to the handheld Nano, then `flight_controller.ino` to the airframe Nano.

### Sensor Calibration & Setup
1. *MPU6050*: Keep the quad absolutely still on a level surface during the audible calibration tone at power-up (~4 seconds). The firmware samples gyro/accel offsets and stores them in RAM each boot.
2. *MS5611*: Automatically calibrates baseline pressure on boot. Power on the drone at the takeoff location to keep reference altitude correct.
3. *ESCs*: Ensure ESCs are calibrated to the min/max pulse range (1–2 ms). If needed, temporarily modify `ESC_MIN_US`/`ESC_MAX_US` in the flight controller sketch to match your ESC specifications.

### Radio Link & Telemetry
- Default RF24 addresses: control uplink uses `CTRL1`, telemetry downlink uses `STAT1` (defined in both sketches).
- Data rate is `250 kbps` on channel `115`. Adjust if you encounter local interference but keep both sketches consistent.
- Telemetry prints to the RC Nano’s serial monitor at 115200 baud (altitude, attitude, estimated battery voltage placeholder). Connect USB to the transmitter to view live data.

### Arming & Flight Flow
1. **Pre-flight checks**
   - Verify props removed for bench testing.
   - Power the transmitter first, then the quad.
   - Wait for the buzzer to stop (IMU calibration complete).
   - Confirm telemetry streaming in the transmitter serial console (optional).
2. **Arming**
   - Ensure throttle is at minimum.
   - Engage `SW1` (mapped as the arm switch). The status LED on the quad turns solid and the buzzer silences.
3. **Throttle & attitude control**
   - Left joystick vertical: throttle (0–100% mapped to 1000–2000 µs).
   - Left joystick horizontal: yaw rate.
   - Right joystick vertical: pitch angle (forward/back).
   - Right joystick horizontal: roll angle (left/right).
   - Pots `A6/A7` are transmitted and currently unused in the flight controller; repurpose for tuning (gain, camera tilt, etc.).
4. **Failsafe**
   - If the flight controller misses radio packets for >0.5 s it disarms and sounds the buzzer.
   - Disengage `SW1` to force disarm; motors drop to minimum and PID integrators reset.
5. **Landing**
   - Reduce throttle to minimum.
   - Disarm via `SW1`.
   - Power down the quad, then the transmitter.

### Tuning Tips
- PID defaults (`roll/pitch`: 3.5/1.2/0.04, `yaw`: 2.0/0.8/0.0) are conservative. Adjust in `flight_controller.ino`.
- To expose live trims, map one of the potentiometers inside the loop where setpoints are computed.
- Ensure the frame orientation matches: front = motor on `D3`, right = `D5`. Swap motor outputs in code if your wiring differs.

### Safety
- Calibrate with props removed.
- Add a LiPo voltage divider to `A0` (or another analog pin) and replace the placeholder battery estimator to avoid over-discharge.
- Use proper 3.3 V regulation for the NRF24L01 on both boards; brown-outs will trigger failsafe.

Enjoy your first flights, and iterate on PID gains and radio ergonomics as you gain confidence. Log telemetry over USB to fine-tune the response before flying aggressively.
