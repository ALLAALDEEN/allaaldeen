# Arduino Nano Quadcopter Project

This project provides firmware for a DIY quadcopter built around two Arduino Nano boards communicating over nRF24L01 radios. One Nano runs the flight controller, handling stabilization from an MPU6050 IMU and MS5611 barometer. The second Nano is a handheld RC transmitter that reads dual joysticks, switches, buttons, and potentiometers, sending pilot commands back to the aircraft.

## Repository Layout

- `flight_controller/flight_controller.ino` – main flight firmware
- `rc_controller/rc_controller.ino` – handheld transmitter firmware

## Required Hardware

| Subsystem | Component | Notes |
|-----------|-----------|-------|
| Flight controller | Arduino Nano, nRF24L01, MPU6050 (INT→D2), MS5611 (GY-63), 4× ESCs, 4× brushless motors, piezo buzzer, status LED | ESC pins: FL→D3, FR→D5, RR→D6, RL→D7 |
| RC controller | Arduino Nano, nRF24L01, 2× analog joysticks, 4× push buttons, 2× toggle switches, 2× potentiometers | Joysticks on A0–A3 + push buttons on A4/A5 |

> **LED pin note:** The original spec listed the status LED on D7, which is also assigned to the rear-left ESC. The firmware drives the status LED from D13 (onboard Nano LED). Move the LED lead to D13 or adjust `LED_PIN` in the flight firmware if you wire it differently.

## Arduino Library Dependencies

Install these via the Arduino IDE Library Manager (Sketch → Include Library → Manage Libraries):

- `RF24` by TMRh20
- `Servo` (bundled with Arduino IDE)

The code implements lightweight drivers for the MPU6050 and MS5611 sensors directly, so no additional libraries are required for those.

## Wiring Reference

### Flight Controller Nano

| Pin | Function |
|-----|----------|
| D2 | MPU6050 INT |
| D3 | Front-left ESC signal |
| D4 | nRF24L01 CE |
| D5 | Front-right ESC signal |
| D6 | Rear-right ESC signal |
| D7 | Rear-left ESC signal |
| D8 | Buzzer |
| D10 | nRF24L01 CSN |
| A4 / A5 | I²C SDA / SCL (MPU6050 + MS5611) |
| A7 | Battery monitor (via voltage divider) |
| 3V3 & GND | nRF24L01 power (add 10 µF capacitor close to radio) |

### RC Transmitter Nano

| Pin | Function |
|-----|----------|
| D2 | Switch 1 (arm) |
| D3 | Switch 2 (mode) |
| D4–D7 | Buttons 1–4 (active LOW) |
| D9 | nRF24L01 CE |
| D10 | nRF24L01 CSN |
| A0 | Throttle joystick vertical |
| A1 | Yaw joystick horizontal |
| A2 | Pitch joystick vertical |
| A3 | Roll joystick horizontal |
| A4 | Left joystick push button |
| A5 | Right joystick push button |
| A6 | Potentiometer 1 |
| A7 | Potentiometer 2 |

## Building & Flashing

1. Open `flight_controller/flight_controller.ino` in the Arduino IDE.
2. Select the correct board (`Arduino Nano`) and processor (`ATmega328P (Old Bootloader)` for clones).
3. Install the libraries listed above.
4. Compile and upload to the flight controller Nano.
5. Repeat for `rc_controller/rc_controller.ino`, uploading to the transmitter Nano.

## Calibration & Setup

1. **Radio smoke test:** Power both boards from USB, open the Serial Monitor at 115200 bps on the transmitter. You should see telemetry lines once both sketches run.
2. **Transmitter stick calibration:** Hold `BUTTON_4` while powering the transmitter to trigger calibration. Follow the Serial Monitor prompts to move each stick through its full range.
3. **Flight controller IMU calibration:** On boot, the controller averages IMU readings for ~6 s. Keep the drone perfectly still during this time.
4. **Barometer calibration:** The flight controller automatically averages altitude after IMU calibration. Keep the craft at the intended “ground level” during this window.
5. **ESC calibration (optional):** If your ESCs support throttle-range learning, comment/uncomment the ESC calibration section you prefer and follow the ESC manufacturer instructions. By default, the sketch sends minimum throttle after boot.
6. **Battery monitor:** Connect a resistor divider to A7 (e.g., 100 kΩ / 33 kΩ) so the pin never sees more than 5 V. Adjust the divider ratio in `sendTelemetry()` if needed.

## Arming & Flight Modes

| Control | Purpose |
|---------|---------|
| Switch 1 (D2) | Arm/disarm (up/on = armed). Motors disarm automatically if radio link drops >300 ms. |
| Switch 2 (D3) | Flight mode placeholder. Currently toggles “angle mode” flag for future expansion. |
| Left joystick vertical | Throttle (0–1000 µs offset) |
| Left joystick horizontal | Yaw rate command |
| Right joystick vertical | Pitch angle command |
| Right joystick horizontal | Roll angle command |

The flight firmware implements a complementary filter for attitude estimation and PID stabilization tuned for a starting point. Expect to retune `pidRoll`, `pidPitch`, and `pidYaw` gains to match your airframe and prop combination.

## First Flight Checklist

1. Remove propellers for early tests. Verify all motor directions and order by gently raising throttle.
2. Confirm radio failsafe by powering off the transmitter – all motors must stop.
3. Verify orientation: with props removed and throttle low, tilt the frame and ensure opposite motors speed up to counteract the tilt.
4. Mount propellers only after direction and stabilization check passes.
5. Arm with Switch 1 up, raise throttle smoothly, and keep the craft low to tune PID gains.
6. Adjust PID values in `flight_controller.ino` between test flights. Reduce `kp` or `kd` if oscillations occur; increase `kp` if the craft feels sluggish.

## Flying Tips

- Always start on a wide, open field with minimal wind.
- Keep Switch 2 off (angle mode) until you are comfortable; you can later extend the code to support acro mode or altitude hold.
- Use the transmitter potentiometers to experiment with auxiliary functions—e.g., map one to camera tilt in future code updates.
- Land immediately if the buzzer emits a steady tone (indicates disarm or failsafe).

## Troubleshooting

- **No radio link:** Double-check CE/CSN wiring and ensure both sketches use matching `RADIO_ADDRESS`, channel, and data rate. Add a 10 µF capacitor across nRF24L01 3.3 V and GND.
- **Drifting or sluggish response:** Re-run stick calibration, ensure props are balanced, and revisit IMU calibration (keep the drone still on boot).
- **Motor twitching:** Confirm ESC signal ground is shared with the Nano ground. Verify BEC supplies are stable.
- **Altitude estimate noisy:** Add foam around the barometer to shield it from prop wash. Increase sampling average or implement a moving average filter.

## Next Steps

- Add OLED or TFT feedback on the transmitter using the telemetry data.
- Implement altitude hold or GPS position hold using the infrastructure already laid out.
- Log flight data over Serial for post-flight analysis.

Fly safe!
