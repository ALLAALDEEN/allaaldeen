# Arduino Nano NRF24L01 Quadcopter Project

This repository contains a simple DIY drone stack built around two Arduino Nano boards communicating over NRF24L01 modules:

- `src/rc_transmitter/rc_transmitter.ino` — handheld radio controller with dual joysticks, switches, buttons, and potentiometers.
- `src/flight_controller/flight_controller.ino` — quadcopter flight controller with MPU6050 IMU, MS5611 barometer, and ESC outputs.

Both sketches speak the same binary packet format and are designed to be uploaded directly from the Arduino IDE (or PlatformIO after minor adjustments).

---

## Hardware Overview

### Aircraft (Flight Controller)

| Subsystem | Part | Connection |
|-----------|------|------------|
| MCU | Arduino Nano (ATmega328P) | USB for programming, VIN/5 V for power |
| Radio | NRF24L01 | CE → D4, CSN → D10, SCK → D13, MOSI → D11, MISO → D12, VCC → 3.3 V with decoupling |
| IMU | MPU6050 (INT) | SDA → A4, SCL → A5, INT → D2, VCC → 5 V |
| Barometer | MS5611 (GY-63) | SDA → A4, SCL → A5, VCC → 5 V |
| Motors / ESCs | Quad layout | FL → D3, FR → D5, RR → D6, RL → D7 (PWM outputs) |
| Buzzer | Active piezo | Signal → D8, GND → GND |
| Status LED | Any LED + resistor | Recommended: signal → D13 (D7 is already occupied by RL motor output) |

> ⚠️ The original spec listed an LED on D7, but that pin is needed for the rear-left ESC signal. Move the status LED to D13 (or any unused pin) before powering the motors.

### Handheld RC Transmitter

| Control | Connection |
|---------|------------|
| NRF24L01 | CE → D9, CSN → D10, SPI pins shared (D11/D12/D13) |
| Left joystick | Vertical → A0 (Throttle), Horizontal → A1 (Yaw), Push → A4 |
| Right joystick | Vertical → A2 (Pitch), Horizontal → A3 (Roll), Push → A5 |
| Buttons 1–4 | D4, D5, D6, D7 (INPUT_PULLUP) |
| Toggle switches | SW1 → D2, SW2 → D3 (pull to GND to activate) |
| Pots | POT1 → A6, POT2 → A7 |

Power the handheld from a 2S LiPo with a 5 V regulator or a USB power bank; keep the NRF24L01 supplied at 3.3 V with adequate decoupling (≥10 µF + 0.1 µF close to the module).

---

## Required Arduino Libraries

Install the following libraries via the Arduino Library Manager (Sketch → Include Library → Manage Libraries…):

- **RF24** by TMRh20 (for the NRF24L01 link)
- **Servo** (built-in)
- **MPU6050** by Electronic Cats (Jeff Rowberg’s I2Cdev wrapper)
- **MS5611** by Rob Tillaart (or equivalent)

If you use different library variants, adapt the include statements inside the sketches accordingly.

---

## Building and Uploading

1. **Clone / copy this repository** to your local machine.
2. **Open `src/rc_transmitter/rc_transmitter.ino`** in the Arduino IDE.
   - Select `Tools → Board → Arduino AVR Boards → Arduino Nano`.
   - Choose the correct processor (`ATmega328P (Old Bootloader)` if your clone requires it).
   - Upload to the Nano that will live inside the handheld controller.
3. **Open `src/flight_controller/flight_controller.ino`** and repeat the upload steps for the airframe Nano.
4. **Power considerations**:
   - The transmitter Nano can be USB-powered while programming.
   - The flight controller must have its ESC/BEC 5 V rail disconnected from USB during programming to avoid backfeeding; use only USB power or remove the red wire from one ESC while flashing.

---

## Initial Setup & Calibration

1. **Radio link sanity check**
   - Power both boards via USB.
   - Open the Serial Monitor at `115200 baud` for each Arduino.
   - Move the sticks; the transmitter prints current channel values, while the flight controller reports roll/pitch/altitude once packets are received.

2. **IMU calibration**
   - On first boot the flight controller performs a simple gyro/accel calibration (board must remain still for ~5 s).
   - If you need to re-calibrate, reboot the flight controller while keeping it level and stationary.

3. **ESC calibration (optional but recommended)**
   - Disconnect props.
   - Upload a basic ESC-calibration sketch or temporarily modify `initMotors()` to send `2000 µs` for 2 s, then `1000 µs`. Perform calibration once per ESC brand.

4. **LED & buzzer feedback**
   - Short single beep on power-up.
   - Double beep whenever the controller disarms (loss of signal or safety trigger).
   - LED solid when armed and receiving valid packets.

---

## Control Mapping

| Control | Function |
|---------|----------|
| Left stick (vertical, A0) | Throttle (0 → min, 1023 → max) |
| Left stick (horizontal, A1) | Yaw rate (±180°/s) |
| Right stick (vertical, A2) | Pitch angle (±25° target) |
| Right stick (horizontal, A3) | Roll angle (±25° target) |
| Switch `SW1` (D2) | Arm/disarm (HIGH = armed when throttle is low) |
| Switch `SW2` (D3) | Reserved/auxiliary (sent in packet bit 1) |
| Buttons 1–4 | Sent as bitfield, available for future flight-modes |
| POT1 (A6) | Altitude trim/damping (adds/subtracts throttle microseconds) |
| POT2 (A7) | Free channel for custom features |

Failsafe: if no valid packet arrives for `>300 ms`, the flight controller disarms, cuts the motors, and beeps twice.

---

## How to Fly

1. **Pre-flight**
   - Remove propellers during bench testing.
   - Verify that all stick directions shown on Serial Monitor match the intended channel (reverse wiring or adjust `map()` if necessary).
   - Mount the flight controller rigidly with the arrow pointing forward; motors should follow the standard X configuration:  
     - Front Left (D3) → CCW  
     - Front Right (D5) → CW  
     - Rear Right (D6) → CCW  
     - Rear Left (D7) → CW  
   - Ensure props match the rotation direction.

2. **Arming sequence**
   - Place the drone on a level surface.
   - Power the airframe, then the transmitter.
   - Keep throttle at minimum.
   - Flip `SW1` to the ON position; you should hear a short confirmation beep and see the status LED light up.

3. **Takeoff**
   - Slowly raise throttle until the quad lifts.
   - Use the right stick to keep the craft level (roll/pitch) and the left stick horizontal axis to manage yaw.

4. **In-flight adjustments**
   - Use POT1 to offset throttle slightly if the quad tends to rise or sink.
   - Use POT2 or spare buttons to trigger your own future features (camera gimbal, lights, etc.).

5. **Landing & disarming**
   - Gradually lower throttle.
   - After touchdown, keep throttle low and toggle `SW1` OFF to disarm—double beep confirms motor cutoff.
   - Only remove power once the controller is disarmed.

> **Safety reminders**  
> - Always test without propellers until you are confident in the control response.  
> - Never arm indoors or near people/animals.  
> - Secure the NRF24L01 modules with shielded cables if you experience noise or brown-outs.

---

## Extending the Project

- Tune PID gains (`pidRoll`, `pidPitch`, `pidYaw`) according to your airframe weight and motor response.
- Implement advanced flight modes (angle/acro, altitude hold) by using spare buttons/switches.
- Add telemetry by opening a second NRF24L01 pipe or using Serial to Bluetooth.
- Integrate GPS (Neo-6M) for position hold once the basics are stable.

Contributions, tuning reports, or questions are welcome—open an issue or PR with your improvements.
