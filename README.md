# DIY NRF24 Quadcopter Project

This project contains the firmware for a DIY quadcopter based on two Arduino Nano boards communicating over NRF24L01 radios. The workspace is split into:

- `flight_controller/src/Flight_Controller.ino` &mdash; runs on the quadcopter airframe, fusing IMU and barometer data, stabilizing the craft with PID control, and driving the ESCs.
- `rc_controller/src/RC_Transmitter.ino` &mdash; runs on the handheld radio controller, reading sticks, switches, buttons, and potentiometers, and streaming commands to the aircraft.
- `docs/` &mdash; build notes, wiring guides, and operating procedures.

> ⚠️ **Safety first:** Spinning propellers are dangerous. Always remove props when testing electronics on the bench. Double-check wiring and polarities, and never fly near people or property you cannot afford to damage.

---

## Hardware Overview

### Flight Controller (Quadcopter)

| Component            | Connection(s)                                          |
|----------------------|--------------------------------------------------------|
| Arduino Nano         | Master MCU                                             |
| NRF24L01             | CE → `D4`, CSN → `D10`, SCK → `D13`, MOSI → `D11`, MISO → `D12`, Vcc → 3.3 V with decoupling capacitor |
| MPU6050 IMU          | SDA → `A4`, SCL → `A5`, INT → `D2`, Vcc → 5 V          |
| GY-36 (MS5611)       | SDA → `A4`, SCL → `A5`, Vcc → 5 V                      |
| ESC Front-Left       | Signal → `D3`                                          |
| ESC Front-Right      | Signal → `D5`                                          |
| ESC Rear-Right       | Signal → `D6`                                          |
| ESC Rear-Left        | Signal → `D7` (share carefully with any LED indicator) |
| Buzzer               | `D8` (active high)                                     |
| Status LED           | Built-in LED on `D13` (optional external)              |

> Note: The original plan listed an LED on `D7`, which conflicts with the rear-left motor output. Use the built-in LED on `D13` or move the LED to a spare pin if you need an external indicator.

### RC Transmitter

| Component            | Connection(s)                                          |
|----------------------|--------------------------------------------------------|
| Arduino Nano         | Master MCU                                             |
| NRF24L01             | CE → `D9`, CSN → `D10`, SPI shared with Nano           |
| Left Joystick        | Vertical (Throttle) → `A0`, Horizontal (Yaw) → `A1`, Push → `A4` (active low) |
| Right Joystick       | Vertical (Pitch) → `A2`, Horizontal (Roll) → `A3`, Push → `A5` (active low) |
| Buttons              | BTN1 → `D4`, BTN2 → `D5`, BTN3 → `D6`, BTN4 → `D7` (wired to pull low) |
| Toggle Switches      | SW1 → `D2`, SW2 → `D3` (wired to pull low)             |
| Potentiometers       | POT1 → `A6`, POT2 → `A7`                               |
| Power                | 2S LiPo + 5 V regulator or USB power pack             |

---

## Required Arduino Libraries

Install the following libraries through the Arduino IDE **Library Manager**:

- **RF24** by TMRh20 (NRF24L01 driver)
- **Servo** (bundled with Arduino IDE)

The code uses raw `Wire` calls for the MPU6050 and MS5611, so no extra sensor libraries are required.

---

## Flashing the Firmware

1. Open `rc_controller/src/RC_Transmitter.ino` in the Arduino IDE and select the correct board/port.
2. Upload to the Nano used in the handheld transmitter.
3. Open `flight_controller/src/Flight_Controller.ino` and upload to the Nano on the quadcopter frame.
4. After flashing, power-cycle both boards so they start cleanly.

---

## Sensor & ESC Calibration

1. **MPU6050 Gyro/Accel Bias**  
   The flight controller automatically samples 2000 IMU readings on boot. Place the quad on a perfectly still, level surface during boot and let the calibration finish (buzzer will stay silent; status printed over USB serial).

2. **MS5611 Barometer**  
   No manual calibration needed, but perform the first power-on at the take-off site to capture the correct pressure baseline.

3. **ESC Calibration**  
   - Disconnect props.
   - Power the quad without arming (leave SW1 off).  
   - Hold throttle stick full up, toggle SW1 on, and connect the main flight battery.  
   - Wait for ESC beeps indicating max throttle, then move throttle to minimum.  
   - ESCs will play confirmation tones. Power-cycle and reattach props.

---

## Radio Link & Arming Logic

- The transmitter sends packets at 100 Hz. Each payload contains stick positions mapped to standard RC pulse widths (1000–2000 µs), button/switch bitfields, and a checksum.
- The flight controller validates the checksum and treats the packet as fresh input. If no valid packets arrive for 500 ms, failsafe triggers: motors stop, buzzer beeps, and `armed` state resets.
- **Arming:**  
  - Throttle below 1050.  
  - Toggle switch `SW1` (transmitter `D2`) to the **active** position (pulled low).  
  - You will hear a short arming tone and the status LED will turn on.
- **Disarming:**  
  - Lower throttle to minimum OR toggle `SW1` off.  
  - Motors stop immediately and a disarm tone plays.

`SW2` and the joystick buttons/pots are free for future modes (acro, altitude hold, etc.).

---

## Understanding the Sensors

- **MPU6050 IMU:**  
  Combines a 3-axis gyroscope (measures rotation rates) and a 3-axis accelerometer (measures direction of gravity). The flight controller fuses both through a complementary filter: the gyro tracks rapid motion, while the accelerometer corrects long-term drift, yielding stable pitch/roll angles.
- **MS5611 Barometer:**  
  High-resolution pressure sensor used to estimate altitude. In this firmware it is sampled at 20 Hz and logged over serial; future upgrades can use it for altitude hold.

---

## How to Fly

1. **Pre-flight Checklist**
   - Props removed for bench tests; install only for flight-ready setup.
   - All screws, arms, and prop nuts secure.
   - Battery charged and securely mounted.
   - Transmitter turned on first; verify telemetry over USB serial during early tests.
   - Quad placed on level ground, perfectly still during boot/calibration.

2. **Arming Sequence**
   - Ensure throttle stick is fully down.
   - Flip `SW1` to arm; listen for tone and confirm motors are idle but armed.
   - Slowly raise throttle to spool up motors. Keep inputs gentle; let the PID stabilize.

3. **Stick Functions**
   - **Throttle (Left stick up/down):** Controls collective thrust for climb/descent.
   - **Yaw (Left stick left/right):** Spins the quad around the vertical axis.
   - **Pitch (Right stick up/down):** Tilts forward/back to move forward/backward.
   - **Roll (Right stick left/right):** Tilts left/right for lateral movement.

   The firmware maps stick deflection to ±25° pitch/roll setpoints by default. Adjust gains or mapping for more/less aggressive response.

4. **Hover Practice**
   - Start with small throttle increases until the quad becomes light on the skids.
   - Maintain altitude by balancing throttle and minimal pitch/roll inputs.
   - Use yaw sparingly at first; small corrections keep orientation manageable.

5. **Emergency Procedures**
   - Drop throttle to minimum to disarm quickly in a tip-over.
   - If the link drops (buzzer sounds), motors stop automatically. Investigate before rearming.

6. **Post-Flight**
   - Disarm before touching the quad.
  - Disconnect the flight battery, then turn off the transmitter.
   - Inspect for loose components, warm motors, or damaged props.

---

## Tuning Tips

- Start with the default PID gains. If the craft oscillates, lower the `kp` for the affected axis. If it feels sluggish, increase `kp` slightly.
- `ki` corrects long-term drift; increase gradually if the quad leans over time, but keep it small to avoid slow oscillations.
- `kd` dampens overshoot. If bounce-back occurs after stick releases, increase `kd` a bit.
- Adjust complementary filter alpha (`COMPLEMENTARY_ALPHA`) if you need more accelerometer influence (lower value) or more gyro dominance (higher).

---

## Repository Layout

```
flight_controller/
  src/Flight_Controller.ino
rc_controller/
  src/RC_Transmitter.ino
docs/
  (add your build photos, wiring diagrams, tuning logs here)
```

---

## Next Steps

- Log data over serial during test hovers to refine PID gains.
- Consider adding altitude hold or acro mode triggered by `SW2`.
- Shield the NRF24 modules from ESC noise and keep antennas clear of carbon fiber for best range.

Fly safe and have fun building! 🚁
