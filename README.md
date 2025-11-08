# DIY NRF24L01 Quadcopter

This project provides a complete open-source stack for a DIY quadcopter built around two Arduino Nano boards communicating over nRF24L01+ radios. It includes:

- A flight controller sketch that fuses MPU6050 IMU data with MS5611 barometric pressure, computes PID stabilisation, and drives four ESCs plus status outputs.
- A handheld RC transmitter sketch that reads dual joysticks, switches, buttons, and potentiometers, then streams commands (with ack-based telemetry) back and forth over the 2.4 GHz link.
- Documentation covering wiring, calibration, safety, and flying procedures so you can assemble, tune, and fly the platform.

> **Note:** The original wiring list mapped both the rear-left motor and status LED to digital pin D7 on the flight controller. Because a single Arduino pin cannot independently control an ESC and an LED, the code defaults the status LED to the on-board LED at D13. If you need an external LED, reassign it to a free digital pin (e.g. D12) and update `PIN_STATUS_LED` in `flight_controller.ino`.

---

## Repository Layout

- `flight_controller/flight_controller.ino` – core flight firmware (IMU/barometer fusion, PID control, ESC output, failsafes, telemetry).
- `rc_controller/rc_controller.ino` – handheld transmitter (stick calibration, packet formation, telemetry display).
- `shared/rc_protocol.h` – shared packet definitions and CRC utilities for both sketches.

---

## Required Hardware

### Flight Controller Stack

| Component | Notes / Connections |
|-----------|---------------------|
| Arduino Nano | ATmega328P @ 16 MHz |
| nRF24L01+ | CE → D4, CSN → D10, SCK → D13, MOSI → D11, MISO → D12, VCC → 3.3 V (with decoupling), GND |
| MPU6050 | SDA → A4, SCL → A5, INT → D2, VCC → 3.3–5 V (with level shifting if required) |
| GY-36 (MS5611) | SDA → A4, SCL → A5, VCC → 3.3 V, GND |
| ESC / Motor FL | Signal → D3 |
| ESC / Motor FR | Signal → D5 |
| ESC / Motor RR | Signal → D6 |
| ESC / Motor RL | Signal → D7 |
| Buzzer | Positive → D8 (through NPN or driver if buzzer >20 mA), Negative → GND |
| Status LED | Default: Arduino on-board LED (D13). External LED optional. |
| Power | 5 V regulator for Nano & sensors; ESC BEC can be used if stable. |

### RC Transmitter

| Component | Notes / Connections |
|-----------|---------------------|
| Arduino Nano | |
| nRF24L01+ | CE → D9, CSN → D10, SPI as above |
| Left joystick | V (vertical) → A0 (throttle), H (horizontal) → A1 (yaw), SEL → A4 (with pull-up, active low) |
| Right joystick | V → A2 (pitch), H → A3 (roll), SEL → A5 |
| Push buttons | D4–D7 (use pull-up, active low) |
| Toggle switches | SW1 → D2, SW2 → D3 (use pull-up, active low) |
| Potentiometers | POT1 → A6, POT2 → A7 |
| Power | 5 V supply or LiPo (with regulator); keep radio on clean 3.3 V rail |

---

## Software Dependencies

Both sketches compile under the Arduino IDE or PlatformIO for the Arduino Nano (ATmega328P):

- **Core libraries:** `Wire`, `SPI`, `Servo`, `EEPROM` (ships with Arduino).
- **External library:** [`RF24` by TMRh20](https://github.com/nRF24/RF24).

Install `RF24` once via the Arduino Library Manager (`Sketch → Include Library → Manage Libraries…`).

---

## Building & Uploading

1. Install the Arduino IDE (2.x recommended) or PlatformIO.
2. Install the `RF24` library.
3. Open `flight_controller/flight_controller.ino`, select **Board:** `Arduino Nano` and **Processor:** `ATmega328P (Old Bootloader)` if required by your board revision.
4. Upload the sketch to the flight controller Nano.
5. Open `rc_controller/rc_controller.ino` and upload it to the transmitter Nano.

> Tip: Label your USB cables to avoid mixing up the two boards during iterations.

---

## Calibration Workflow

### RC Transmitter

1. **First boot (or press Button 4 while powering):** The transmitter enters calibration mode.
2. Follow the serial prompts (open Serial Monitor @ 115200 baud):
   - Keep sticks centered when requested to capture center values.
   - Move each stick through full travel during the 5-second windows.
3. Calibrations persist in EEPROM; repeat if you change hardware or feel drift.
4. Button mapping (also mirrored inside the packets):
   - Button 1 → Arm (sets `BUTTON_ARM`).
   - Button 2 → Disarm.
   - Button 3 → Buzzer (request audible locator on drone).
   - Button 4 → Calibration override on power-up.
   - Stick presses → additional momentary inputs (bits exposed in packets for future use).

### Flight Controller

1. Place the airframe on a level surface.
2. Power on the drone (flight controller + peripherals).
3. IMU calibration runs automatically (~1.6 s) while the craft must stay still.
4. Barometer baseline averages ~1 s of pressure data; avoid drafts.
5. Status tones:
   - Rising double beep → boot complete, ready to arm.
   - Single beep when arming/disarming transitions.

---

## Failsafe & Arming Logic

- Radio packets include a CRC16; invalid packets are discarded.
- If no valid packet has been received for 500 ms, the controller cuts throttle, disarms, and pulses the buzzer.
- Arming requires:
  - Valid link (acknowledged packets).
  - Arm command (Button 1) held.
  - Throttle below 1050 µs.
- Disarming occurs if:
  - Button 2 pressed, **or**
  - Radio link lost (failsafe), **or**
  - Flight controller detects arm command released without new confirmation.

---

## Control Mapping

| Axis | Source (transmitter) | Destination (flight controller) | Range |
|------|----------------------|---------------------------------|-------|
| Throttle | Left joystick vertical (A0) | ESC baseline | 1000–2000 µs |
| Yaw | Left joystick horizontal (A1) | PID yaw target rate | −150°/s to +150°/s |
| Pitch | Right joystick vertical (A2) | PID pitch angle target | −25° to +25° |
| Roll | Right joystick horizontal (A3) | PID roll angle target | −25° to +25° |
| SW1 | Toggle 1 (D2) | `SWITCH_MODE` (future use) | Binary |
| SW2 | Toggle 2 (D3) | Altitude hold enable | Binary |
| POT1 | A6 | Packet `aux1` / telemetry echo | 0–1000 |
| POT2 | A7 | Packet `aux2` / telemetry echo | 0–1000 |

---

## Sensor Fusion Overview

- **MPU6050 Gyroscope** measures rotational rates around X/Y/Z. The controller integrates these to maintain an estimate of how quickly the craft rotates, supplying the PID derivative component.
- **MPU6050 Accelerometer** measures gravity along three axes. A complementary filter blends accelerometer angles with integrated gyro data to produce stable pitch/roll estimates:
  ```
  attitude = (1 − α) * (attitude + gyro * dt) + α * accel_angle
  ```
  with `α ≈ 0.04` giving fast gyro response while slowly correcting drift.
- **MS5611 Barometer** measures absolute pressure at high resolution. After calibration to the take-off pressure, altitude is derived via the International Standard Atmosphere formula. A dedicated altitude PID engages when SW2 is on, holding the captured height.

---

## Pre-Flight Checklist

1. **Airframe & wiring**
   - Confirm motor directions: FL & RR CCW, FR & RL CW (adjust ESC leads).
   - Secure the MPU6050 flat, vibration-isolated (foam tape).
   - Shield the MS5611 from prop wash.
   - Provide a clean 3.3 V supply to the nRF24L01+ (with a 10 µF cap close to VCC/GND).
2. **Radio link**
   - Power the transmitter first, verify telemetry output in Serial Monitor (`Telemetry | …`).
   - Power the quad; confirm arming LED / tone ready state.
3. **Calibration**
   - Ensure prior calibrations are recent; recalibrate if hardware changed.
4. **Propellers off** during bench tests. Only attach props once arming, motor mixing, and failsafe behaviour are verified.

---

## Flying the Drone

1. **Power sequence:** Turn on the transmitter → connect the flight battery.
2. **Wait for ready tone:** The flight controller double-beeps after IMU/baro calibration.
3. **Arm:**
   - Ensure throttle stick fully down.
   - Press and hold Button 1 (Arm). The buzzer chirps, LED lights.
4. **Lift-off:**
   - Raise throttle smoothly to ~40–50% to get light on the skids, then continue to break ground.
   - Use the right stick to maintain level attitude; use left stick horizontal to manage yaw.
5. **Altitude hold (optional):**
   - While hovering steadily, flip SW2 to ON. The barometer target locks at current height.
   - Throttle commands become trim changes via the altitude PID; flip SW2 OFF to return to manual control.
6. **Flight:**
   - Roll right stick left/right for lateral movement.
   - Pitch forward/back to move forward/back.
   - For coordinated turns, combine yaw (left stick horizontal) with slight roll.
7. **Disarm / Land:**
   - Reduce throttle gently and descend.
   - On touchdown, pull throttle fully low and press Button 2 (Disarm). Motors stop and LED turns off.
   - Disconnect flight battery; switch off transmitter last.

> **Safety:** Always stand behind the front of the drone, wear eye protection, and keep a clear flight area. Practice in a large open space and start with short hops before aggressive manoeuvres.

---

## Tuning Tips

- PID coefficients (`pidRoll`, `pidPitch`, `pidYaw`, altitude PID) are defined near the top of `flight_controller.ino`. Start with defaults; adjust `kp` for responsiveness, `kd` for damping, `ki` for drift correction.
- Use the Serial Monitor on the transmitter to observe telemetry (roll/pitch/altitude) and detect oscillations or drift.
- `SWITCH_MODE` (SW1) is reserved for future flight modes (e.g., beginner/acro). Extend the code by checking `controlPacket.switches & SWITCH_MODE`.
- The transmitter stores stick calibration in EEPROM. If you flash new firmware and experience input clipping, re-run calibration (Button 4 on power-up).

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Motors twitch but do not spin | Throttle not armed, ESCs uncalibrated | Arm with Button 1 at low throttle; calibrate ESC endpoints individually if needed |
| Drone tips on take-off | Incorrect motor order or prop direction | Verify wiring (FL→D3, FR→D5, RR→D6, RL→D7) and prop orientation |
| No telemetry in transmitter Serial Monitor | Radio mismatch, wiring, or no ack payload | Ensure both sketches use same addresses & channel (90); check 3.3 V supply and CE/CSN pins |
| Barometer altitude jumps | Air turbulence around MS5611 | Shield sensor, add foam cover, retune altitude PID |
| Arming fails despite button press | Throttle not fully low or radio not linked | Check transmitter calibration, confirm ack prints when packets arrive |

---

## Extending the Project

- Add OLED or TFT to the transmitter to show telemetry without a computer.
- Log IMU and PID data via Serial or SD card for post-flight analysis.
- Implement additional flight modes (rate/acro, GPS hold) by expanding switch logic.
- Use `aux1`/`aux2` to transmit battery voltage or camera gimbal commands.

---

Happy flying! Carefully test changes with props removed, keep logs of tuning adjustments, and iterate gradually to achieve a smooth, stable quadcopter. If you run into issues, capture Serial logs from both boards—they’re invaluable for diagnosing sensor or radio problems.
