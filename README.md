# DIY NRF24 Quadcopter Project

This repo contains a complete NRF24-based RC link for a DIY quadcopter built around two Arduino Nano boards:

- `flight_controller/flight_controller.ino` – runs on the airframe, handles IMU fusion, barometer, PID stabilization, motor mixing, failsafe logic, and NRF24 reception.
- `rc_transmitter/rc_transmitter.ino` – runs on the handheld radio transmitter, reads joysticks/buttons/switches/pots, packages the data, and pushes it over NRF24.

Both sketches use standard Arduino core libraries plus the [`RF24` library by TMRh20](https://github.com/nRF24/RF24).

## Hardware Wiring

### Airframe (Flight Controller Nano)

| Function            | Pin |
|---------------------|-----|
| NRF24L01 CE         | D4  |
| NRF24L01 CSN        | D10 |
| NRF24L01 MOSI/MISO/SCK | D11/D12/D13 (hardware SPI) |
| MPU6050 SDA/SCL     | A4/A5 (I²C) |
| MPU6050 INT         | D2 |
| MS5611 SDA/SCL      | A4/A5 (shared I²C) |
| Motor Front-Left    | D3 |
| Motor Front-Right   | D5 |
| Motor Rear-Right    | D6 |
| Motor Rear-Left     | D7 |
| Buzzer              | D8 |
| Status LED          | D13 (built-in LED recommended to avoid D7 conflict) |

> ⚠️ Pin D7 is already used for the rear-left ESC signal. If you need an external status LED, connect it to D13 or any other free pin and update `PIN_STATUS_LED` in the sketch.

### Handheld RC Transmitter Nano

| Function                 | Pin |
|--------------------------|-----|
| NRF24L01 CE              | D9  |
| NRF24L01 CSN             | D10 |
| Left joystick vertical (Throttle) | A0 |
| Left joystick horizontal (Yaw)    | A1 |
| Left joystick push (SEL)          | A4 |
| Right joystick vertical (Pitch)   | A2 |
| Right joystick horizontal (Roll)  | A3 |
| Right joystick push (SEL)         | A5 |
| Push buttons 1–4                  | D4, D5, D6, D7 (active low) |
| Toggle switch 1 (Arm/Disarm)      | D2 (active low) |
| Toggle switch 2 (Altitude hold)   | D3 (active low) |
| Potentiometers 1 & 2              | A6, A7 |
| Status LED (optional)             | D13 |

Wire joystick/button switch commons to GND so the inputs read `LOW` when pressed / ON.

## Library Dependencies

Install the following through the Arduino Library Manager (Sketch → Include Library → Manage Libraries):

- **RF24** by TMRh20 (latest stable release)

The flight controller sketch only uses Arduino core libraries (`Wire`, `SPI`, `Servo`, `tone`). No third-party IMU or barometer library is required.

## Building and Uploading

1. Install the Arduino IDE (1.8.x or 2.x).
2. Add the RF24 library.
3. Open `flight_controller/flight_controller.ino` and select **Arduino Nano** with the correct processor (ATmega328P or ATmega328P (Old Bootloader)) and the right COM port. Upload to the quadcopter Nano.
4. Open `rc_transmitter/rc_transmitter.ino` and upload it to the transmitter Nano.

## Configuration Highlights

### Control Packet Protocol

Both sketches share the same `ControlPacket` structure:

- `throttle`: 0–1000
- `roll`, `pitch`, `yaw`: ±500 (mapped to ±35° tilt for roll/pitch, ±180 °/s for yaw)
- `aux1`, `aux2`: ±500 (mapped to ±1.0 range – used for future tuning)
- `buttons`: bit mask (joystick presses, push buttons, arm switch, altitude hold switch)
- Simple checksum to detect corrupt packets; 250 ms failsafe timeout on the flight controller.

### Flight Controller Key Features

- 250 Hz main loop with complementary-filter attitude estimate from MPU6050.
- MS5611 barometer for basic altitude estimate and optional altitude hold (`SW2`).
- PID loops (default gains tuned for a small 450-class quad). Adjust `pidRoll`, `pidPitch`, `pidYaw`, and `pidAltitude` to suit your frame.
- Motor mixing expects an X-configuration:
  - Front-left (CW), Front-right (CCW), Rear-right (CW), Rear-left (CCW).
- Failsafe kills throttle, disarms, and beeps if no packets arrive for >250 ms.
- Arming logic: `SW1` must be **ON** and throttle below 5% to arm; switching `SW1` OFF disarms immediately.
- Buzzer tones:
  - Startup double-beep on boot.
  - Short chirp when arming/disarming.
  - Periodic alarm on failsafe or when disarmed.

### Transmitter Features

- 100 Hz packet rate.
- Joystick calibration on boot (keep sticks centered for the first second).
- Adjustable channel inversion constants near the top of the sketch (`THROTTLE_INVERT`, `PITCH_INVERT`, etc.).
- `POT_1` and `POT_2` currently map to `aux1/aux2` (±500). You can use them for in-flight PID tuning or camera gimbals later.
- Status LED slow blink = link OK, fast blink = packet retries/failures.

## Initial Setup and Calibration

1. **ESC calibration** (one by one or with a programming card). Ensure every ESC accepts 1000–2000 µs inputs.
2. **Quadcopter power-up**:
   - Remove propellers for the entire setup stage.
   - Power the flight controller with a stable supply (USB or BEC).
   - Keep the airframe perfectly still for the first 5 seconds so the gyro/accelerometer and barometer calibrate properly.
3. **Transmitter power-up**:
   - Hold sticks centered while powering the transmitter so it records joystick offsets.
   - Ensure `SW1` (arm) is **OFF** before connecting the LiPo.
4. **RF link test**:
   - Open Arduino Serial Monitor at 115200 baud on both devices (one at a time) to read status messages.
   - With props off, arm the quad (set `SW1` ON with throttle at minimum). Motors should spin up to idle (~1070 µs).
   - Wiggle sticks and check that motor outputs respond as expected (front-left motor speed increases when pushing pitch forward, etc.).
5. **Altitude sensor check**:
   - Enable `SW2` after arming to latch the current altitude as the hold target. The throttle stick becomes a trim (the code blends altitude correction into the throttle output). Disable `SW2` to return to manual mode.

Once everything behaves correctly, disconnect power and mount the props in the right orientation (two CW, two CCW).

## How to Fly

1. **Pre-flight checklist**
   - Props securely mounted; correct rotation directions.
   - Frame level; battery strapped; wires clear of props.
   - `SW1` OFF, throttle at minimum.
   - Verify transmitter battery is charged.
2. **Power on sequence**
   - Power the transmitter first.
   - Connect the flight battery. Wait for the double-beep and LED heartbeat.
3. **Arming**
   - Place the quad on level ground.
   - Move throttle to minimum.
   - Toggle `SW1` ON. Motors should chirp and spin at idle.
4. **Take-off**
   - Increase throttle smoothly to lift off.
   - Use the right stick for pitch (forward/back) and roll (left/right).
   - Use left stick horizontal for yaw (turning).
5. **Altitude hold (optional)**
   - Once hovering steadily, toggle `SW2` ON to capture the current altitude.
   - Throttle stick becomes a small trim (center = hold). Move it up/down slightly to climb/descend while in hold mode.
   - Switch `SW2` OFF to return to full manual throttle.
6. **Landing**
   - Disable altitude hold if active.
   - Reduce throttle gradually until touchdown.
   - Toggle `SW1` OFF to disarm once the quad is on the ground.
   - Disconnect the flight battery, then turn off the transmitter.

Always test in a wide-open area, away from people and property. Start with very small stick movements until you are comfortable with the responsiveness.

## Tuning Tips

- Adjust PID gains (`kp`, `ki`, `kd`) near the top of `flight_controller.ino`. Start with small changes (±0.005).
- Use `aux1`/`aux2` to implement inflight tuning (e.g., map `aux1` to scale `pidRoll.kp`). The current code reads these values but does not apply them—add your own logic where needed.
- If you get oscillations, reduce `kp` or `kd`. If the quad feels sluggish, increase `kp` slightly.
- Re-run gyro calibration if you notice drifts (power cycle the quad while it is perfectly still).

## Troubleshooting

- **No RF link:** verify wiring (especially CE/CSN), ensure both sketches use the same address/channel, and add a 10 µF capacitor across NRF24 VCC/GND.
- **Quad won’t arm:** check that throttle is below 5%, `SW1` is ON, and the receiver packets are arriving (Serial Monitor should show updates). Also confirm failsafe isn’t triggered.
- **Motor order incorrect:** double-check physical motor wiring vs. the mapping defined at the top of `flight_controller.ino`.
- **Altitude hold unstable:** make sure the MS5611 is connected with short wires, away from prop wash. Consider adding foam over the sensor to dampen airflow.
- **Transmitter inputs reversed:** flip the `*_INVERT` booleans in `rc_transmitter.ino`.

Fly safe! Calibrate thoroughly before installing propellers, and keep clear of rotating parts at all times.
