# DIY NRF24L01 Quadcopter Project

This project contains matching Arduino sketches for a brushed/brushless quadcopter flight controller and an NRF24L01-based RC transmitter. Upload the `flight_controller` sketch to the Arduino Nano that sits on the airframe and the `rc_transmitter` sketch to the handheld radio.

Both sketches are written for the Arduino IDE (or PlatformIO) and assume the **TMRh20 RF24** library is installed.

---

## Hardware Overview

### Flight Controller (Arduino Nano)

| Peripheral              | Pin(s) |
| ----------------------- | ------ |
| NRF24L01 CE / CSN       | D4 / D10 |
| NRF24L01 SPI            | D11 (MOSI), D12 (MISO), D13 (SCK) |
| MPU6050 (GY-521)        | SDA=A4, SCL=A5, INT→D2 |
| GY-63 MS5611 Barometer  | SDA=A4, SCL=A5 |
| Motor ESC FL / FR / RR / RL | D3 / D5 / D6 / D7 |
| Buzzer                  | D8 |
| Status LED              | D13 (recommended; free the D7 motor pin) |

> ⚠️ If you already wired the LED to D7, move it to D13 or another free pin so it does not fight the rear-left ESC signal.

### RC Transmitter (Arduino Nano)

| Control                 | Pin(s) |
| ----------------------- | ------ |
| NRF24L01 CE / CSN       | D9 / D10 |
| Left joystick (Throttle / Yaw) | V→A0, H→A1, Push→A4 |
| Right joystick (Pitch / Roll)  | V→A2, H→A3, Push→A5 |
| Buttons 1–4             | D4, D5, D6, D7 (active LOW) |
| Toggle switches SW1 / SW2 | D2 / D3 (active LOW) |
| Potentiometers 1 / 2    | A6 / A7 |

Power both boards from a clean, regulated 5 V source. Use level shifting or decoupling (10 µF + 100 nF) on each NRF24L01 module.

---

## Software Setup

1. **Install libraries**
   - Install the *RF24 by TMRh20* library from the Arduino Library Manager.
   - The sketches use only core `Wire`, `SPI`, `Servo`, and `math` headers otherwise.

2. **Flight Controller**
   - Open `flight_controller/flight_controller.ino`.
   - Select the Arduino Nano board, the correct processor variant, and the USB port.
   - Upload the sketch.

3. **RC Transmitter**
   - Open `rc_transmitter/rc_transmitter.ino`.
   - Select the second Arduino Nano and upload.

---

## Calibration & Bench Setup

1. **Initial Wiring Check**
   - Keep propellers **off** for all bench tests.
   - Power both boards and confirm the transmitter LED stops blinking (indicates the radio link succeeded).

2. **IMU Calibration**
   - On boot the flight controller lights the LED solid while sampling the MPU6050. Keep the frame perfectly still until it finishes (about 3 s); a short buzzer chirp confirms success.

3. **ESC Calibration (optional but recommended)**
   - Power the quad without props.
   - Hold the throttle stick at maximum, arm switch **off**, then toggle it **on** while powering the flight controller.
   - When the ESCs beep, immediately lower the throttle to minimum. Wait for the completion tones.

4. **RC Joystick Centres**
   - The transmitter samples joystick midpoints during startup. Let both sticks rest at centre while it boots. Move them through full travel once to teach the adaptive min/max tracking.

5. **Failsafe**
   - If the radio link drops for >200 ms the flight controller disarms and pulses the buzzer.

---

## Arming & Flight Controls

- **Arm/Disarm**  
  Flip `SW1` (on the transmitter) to the ON position while the throttle is at minimum. A chirp and solid LED on the quad confirm arming. Flip `SW1` off to disarm.

- **Throttle (Left stick vertical)**  
  Controls total thrust. Stick up increases altitude.

- **Yaw (Left stick horizontal)**  
  Rotates the quad clockwise (right) or counter-clockwise (left).

- **Pitch (Right stick vertical)**  
  Moves the quad forward/backward.

- **Roll (Right stick horizontal)**  
  Slides left/right.

- **Altitude Hold (SW2)**  
  When SW2 is ON and sticks are near centre, the barometer PID keeps altitude. Use POT1 for fine altitude trim while in this mode.

- **Auxiliary Inputs**
  - Buttons and joystick pushes are included in the radio packet for custom features (e.g., camera, lights).
  - POT2 is free for future tuning (e.g., yaw gain send-over).

---

## First Flight Checklist

1. Mount props in an `X` pattern:  
   - **Front-left (D3):** CCW prop  
   - **Front-right (D5):** CW prop  
   - **Rear-right (D6):** CCW prop  
   - **Rear-left (D7):** CW prop
2. Secure all wiring; ensure ESC grounds share the flight controller ground.
3. Power the transmitter first, then the quad.
4. Arm with throttle low, slowly raise throttle to hover.
5. Practice gentle stick inputs before enabling altitude hold.

---

## Troubleshooting

- **No radio link:** Check NRF24L01 orientation, CE/CSN wiring, and add a 10 µF capacitor across VCC/GND.
- **Drift in hover:** Re-run the IMU calibration on a level surface, verify prop directions, inspect frame balance.
- **Motor won’t spin:** Ensure ESC signal ground is connected and the status LED shows armed.
- **Altitude hold oscillates:** Reduce the `pidAltitude` `kp`/`ki` values inside `flight_controller.ino`.

---

Fly safe—always test without props first, wear eye protection, and respect local regulations. Happy flying!
