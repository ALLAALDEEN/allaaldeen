## Drone Flight Controller & RC Transmitter

This repository contains Arduino Nano firmware for a 250‑class quadcopter and its matching handheld RC transmitter. The system uses an nRF24L01 wireless link, an MPU6050 6‑axis IMU, and a GY-36 (MS5611) barometer for assisted stabilization.

### Repository Layout

- `flight_controller/flight_controller.ino` – Stabilisation firmware running on the airframe Nano.
- `rc_controller/rc_controller.ino` – Handheld transmitter firmware.
- `shared/RadioPacket.h` – Shared packet definition used by both sketches.

> **Tip:** When compiling with the Arduino IDE, add `shared/RadioPacket.h` to each sketch folder (or adjust the include to point to your local copy).

### Required Arduino Libraries

Install the following from the Arduino Library Manager (or add manually):

- **RF24** by TMRh20 (nRF24L01 driver)
- **I2Cdevlib MPU6050** by Jeff Rowberg (IMU access)
- **Servo** (bundled with Arduino IDE)

The flight controller sketch ships with a self-contained MS5611 driver, so no extra library is needed for the barometer.

### Hardware Pinout Summary

| Flight Controller Signal | Arduino Nano Pin |
|-------------------------|------------------|
| nRF24L01 CE             | D4               |
| nRF24L01 CSN            | D10              |
| MPU6050 INT             | D2               |
| Motor FL ESC            | D3               |
| Motor FR ESC            | D5               |
| Motor RR ESC            | D6               |
| Motor RL ESC            | D7               |
| Buzzer                  | D8               |
| Status LED              | D13 *(use D7 only if the rear-left motor is moved)* |

| RC Transmitter Signal  | Arduino Nano Pin |
|------------------------|------------------|
| nRF24L01 CE            | D9               |
| nRF24L01 CSN           | D10              |
| Left stick vertical    | A0 (Throttle)    |
| Left stick horizontal  | A1 (Yaw)         |
| Left stick press       | A4               |
| Right stick vertical   | A2 (Pitch)       |
| Right stick horizontal | A3 (Roll)        |
| Right stick press      | A5               |
| Toggle switch 1        | D2               |
| Toggle switch 2        | D3               |
| Button 1               | D4               |
| Button 2               | D5               |
| Button 3               | D6               |
| Button 4               | D7               |
| Potentiometer 1        | A6               |
| Potentiometer 2        | A7               |

### Control Mapping

- **Left stick (vertical)** – Throttle (0–1000).
- **Left stick (horizontal)** – Yaw rate (°/s).
- **Right stick (vertical)** – Pitch angle command (±35°).
- **Right stick (horizontal)** – Roll angle command (±35°).
- **Toggle SW1 (D2)** – Arm/Disarm (up = armed). Keep throttle at zero before arming.
- **Toggle SW2 (D3)** – Altitude hold enable (requires barometer active). Uses POT_1 to set ±3 m range around takeoff altitude.
- **Joystick presses & buttons** – Reserved for future modes (telemetry, beeper, etc.). Currently sent in the packet for easy expansion.

### Building & Uploading

1. **Prepare the sketches**
   - Copy `shared/RadioPacket.h` into both `flight_controller/` and `rc_controller/` folders if your IDE does not support parent includes.
   - Open `rc_controller/rc_controller.ino` in the Arduino IDE and select **Board: Arduino Nano** with the correct processor/USB options. Compile & upload to the handheld transmitter Nano.
   - Open `flight_controller/flight_controller.ino`, select the matching board settings, and upload to the flight controller Nano.

2. **Library verification**
   - In the Arduino IDE, verify that `RF24`, `MPU6050`, and `Servo` libraries are visible under **Sketch → Include Library**.
   - No separate MS5611 library is needed; the driver is built-in.

3. **Power considerations**
   - Power the RC transmitter from a regulated 5 V source or USB battery.
   - Power the flight controller from the quad’s 5 V BEC. Ensure ESC BEC outputs share a ground reference with sensors and radio.

### Calibration & First-Time Setup

1. **IMU calibration**
   - Place the quad on a perfectly level surface.
   - Power on the flight controller and remain still for ~10 seconds. The firmware automatically samples the MPU6050 to compute offsets.

2. **Transmitter stick calibration**
   - When the RC sketch boots, it samples the center position of yaw/roll/pitch sticks and the minimum throttle. Keep sticks centered (throttle fully down) for the first couple of seconds after applying power.

3. **Barometer zeroing**
   - The flight controller averages ~60 initial MS5611 readings to define “ground altitude.” Avoid drafts and prop wash during this phase.

4. **ESC calibration (optional)**
   - Calibrate ESC endpoints using their manufacturer instructions before first flight to guarantee all motors interpret 1000–2000 µs pulses consistently.

### Arming, Disarming & Failsafe

- **Disarmed state** – SW1 down. Motors hold at minimum, LED slow blinks.
- **Arming** – Throttle at zero, flip SW1 up. The buzzer beeps once and the LED becomes solid.
- **Disarming** – Flip SW1 down or lose radio link for >200 ms. The buzzer emits two short beeps and motors spin down.
- **Failsafe** – If radio packets stop, motors reduce to idle for 500 ms, then cut to minimum. A blinking LED indicates failsafe.

### Flying Guide

1. **Pre-flight**
   - Inspect propellers, motor mounts, and frame screws.
   - Verify prop rotation directions and orientation (front-left/right props should be opposite).
   - Secure battery and ensure center of gravity is near the frame center.
   - Power the transmitter first, then the quad.

2. **Takeoff**
   - Arm using SW1 while throttle is zero.
   - Raise throttle smoothly until the quad lifts off. Use gentle right-stick inputs to maintain level attitude.

3. **In-flight control**
   - Use the right stick to command angles; the PID loop holds the requested orientation.
   - Yaw control (left stick horizontal) rotates the quad in place.
   - Enable SW2 to engage altitude hold. POT_1 shifts the target altitude in a ±3 m window around where hold was enabled. Throttle still biases climb/descent when hold is active.

4. **Landing**
   - Disable altitude hold (SW2 down).
   - Gradually reduce throttle while keeping the frame level.
   - Once landed, cut throttle to zero and flip SW1 down to disarm.

5. **Post-flight**
   - Disconnect the flight battery promptly to avoid over-discharge.
   - Inspect for loose screws, warm motors, or damaged props before the next flight.

### Safety Notes

- Never arm the quad indoors or while people/animals are within propeller reach.
- Always remove propellers when performing bench testing or firmware updates.
- Consider adding prop guards and a physical kill switch for extra safety.
- Keep RF antennas clear of carbon fiber and metal parts to avoid range loss.

Happy flying! Calibrate carefully, make small PID adjustments as needed, and incrementally expand capabilities (GPS hold, telemetry, etc.) once the base system is stable.
