# DIY Quadcopter Flight System

This workspace contains two Arduino sketches that together form a basic 2.4 GHz RC quadcopter setup:

- `flight_controller/flight_controller.ino` — runs on the drone-side Arduino Nano, handles attitude estimation, PID motor control, nRF24L01+ telemetry reception, and safety logic.
- `rc_transmitter/rc_transmitter.ino` — runs on the handheld Arduino Nano, reads sticks/switches/pots/buttons and continuously transmits command packets to the aircraft.
- `shared/RcPacket.h` — shared radio packet definition and checksum helper used by both sketches (keep this file alongside the two project folders when copying to your Arduino sketchbook).

Both sketches target Arduino Nano hardware (ATmega328P @ 16 MHz) and require the [TMRh20 RF24](https://github.com/nRF24/RF24) library.

---

## 1. Hardware Summary

### Flight Controller (Quadcopter)

| Subsystem        | Parts & Notes                                   | Arduino Pins                          |
|------------------|--------------------------------------------------|---------------------------------------|
| Radio link       | nRF24L01+                                       | CE `D4`, CSN `D10`, SPI (`D11`, `D12`, `D13`) |
| IMU              | MPU6050 (GY-521 etc.)                           | I²C (`A4`, `A5`), INT `D2`            |
| Barometer        | GY-63 MS5611                                     | I²C (`A4`, `A5`)                      |
| Motors / ESCs    | Quad layout (FL/FR/RR/RL)                       | `D3`, `D5`, `D6`, `D7`                |
| Buzzer (active)  | Piezo or beeper (active low)                    | `D8`                                  |
| Status LED       | Uses built-in LED                               | `D13` *(D7 is occupied by RL motor PWM)* |

> ⚠️ If you must use an external LED, tap the built-in LED pad or move the RL motor to another PWM pin (e.g. `D9`) and update `PIN_MOTOR_RL`.

### Handheld RC Transmitter

| Control          | Arduino Pins |
|------------------|--------------|
| nRF24L01+        | CE `D9`, CSN `D10` |
| Left stick       | Throttle `A0`, Yaw `A1`, press `A4` |
| Right stick      | Pitch `A2`, Roll `A3`, press `A5`   |
| Buttons (1–4)    | `D4`, `D5`, `D6`, `D7` (active low) |
| Switches (SW1/2) | `D2`, `D3` (active low to GND)      |
| Pots (1/2)       | `A6`, `A7`                          |

Power both boards from stable 5 V sources. Add 10 µF capacitors across the nRF24L01+ modules and, ideally, use PA/LNA breakout with a regulated supply + decoupling.

---

## 2. Library & Toolchain Setup

1. Install the **Arduino IDE** (v2 recommended) or use the Arduino CLI.
2. Install required libraries through the Library Manager or git:
   - `RF24` by TMRh20.
   - `Servo` (bundled with the IDE).
   - `I2Cdev` and `MPU6050` from the I2Cdevlib collection (by Electronic Cats / Jeff Rowberg).
   - `MS5611` by Rob Tillaart (or an equivalent that exposes `MS5611.h`).
3. Confirm the sensors share the SDA/SCL bus (`A4/A5`) and keep their default I²C addresses (`MPU6050` = `0x68`, `MS5611` = `0x77`). If you change either address, update the constants in `flight_controller.ino`.
4. Optional (for debugging): `Serial Monitor` at 115200 baud on the flight controller.

---

## 3. Flashing the Sketches

1. Open `rc_transmitter/rc_transmitter.ino` in the Arduino IDE.
2. Select **Board:** “Arduino Nano”, **Processor:** “ATmega328P (Old Bootloader)” if applicable.
3. Upload to the transmitter Nano.
4. Open `flight_controller/flight_controller.ino` and repeat for the flight controller Nano.

> Tip: label your Nanos to avoid swapping sketches later.

---

## 4. Pre-Flight Checklist

1. **Mounting & Orientation**
   - Align the MPU6050 so X-axis points forward, Y to the right, Z upwards.
   - Secure the MS5611 away from prop wash (ideally in a foam-shielded bay).
2. **Power & Wiring**
   - ESC signal grounds must share common ground with the Nano.
   - Solder good-quality power rails and add a power distribution board or harness.
3. **Propellers Off** during configuration and calibration.
4. **Radio Test**
   - Power both boards (without props).
   - Open Serial Monitor on the flight controller (115200 baud). You should see orientation values respond as you move the frame.

---

## 5. Sensor Calibration

Calibration runs automatically at boot while the frame is stationary and level:

1. Place the quadcopter on a perfectly level surface.
2. Power the flight controller. The status LED will stay off during IMU calibration (approx. 3 s).
3. Only after calibration finishes should the transmitter be powered. Move nothing until the LED begins slow blinking (disarmed state with valid link).
4. For improved barometer accuracy, keep the frame at takeoff height for ~10 s after boot so the running altitude baseline settles.

---

## 6. RC Channel Mapping

| Control                   | Packet Field       | Description / Range               |
|---------------------------|--------------------|-----------------------------------|
| Left V (Throttle)         | `throttle`         | `0‒1000` microsecond scaling      |
| Left H (Yaw)              | `yaw`              | `-500‒+500` yaw rate request      |
| Right V (Pitch)           | `pitch`            | `-500‒+500` pitch angle request   |
| Right H (Roll)            | `roll`             | `-500‒+500` roll angle request    |
| Switch SW1 (D2)           | `switches bit0`    | **Arming switch** (1 = arm)       |
| Switch SW2 (D3)           | `switches bit1`    | Flight mode (reserved)            |
| Left stick press (A4)     | `switches bit2`    | Momentary action (user defined)   |
| Right stick press (A5)    | `switches bit3`    | Momentary action (user defined)   |
| Buttons D4‒D7             | `buttons bit0-3`   | Custom functions (e.g. buzzer)    |
| Pots A6/A7                | `aux1`, `aux2`     | Additional tuning channels        |

---

## 7. Arming, Take-off, and Flying

1. **Power-Up Sequence**
   - Power the quadcopter first; wait for calibration (status LED slow blink).
   - Power the transmitter; verify LED becomes steady once SW1 is ON with throttle low.
2. **Arming**
   - Ensure **SW1 is OFF** before applying power.
   - Set throttle to minimum.
   - Flip **SW1 ON**. After ~1 s the status LED will turn solid: motors are armed and ready.
3. **Spool-Up**
   - Slowly raise throttle. Motors should track pitch/roll stick movements smoothly.
4. **In-Flight Controls**
   - Throttle: overall altitude.
   - Roll/Pitch: bank and pitch to maneuver.
   - Yaw: rotate on vertical axis.
   - Pots/buttons: assignable (e.g., PID trimming or flight modes).
5. **Disarming**
   - Land gently.
   - Hold throttle low and flip **SW1 OFF**. The LED returns to slow blink, motors stop.
   - Disconnect battery.

---

## 8. Safety & Tuning Notes

- **Failsafe:** If radio packets stop for >200 ms, motors fall back to idle and the buzzer pulses.
- **RF Channel:** Both sketches default to channel `115` (`RADIO_CHANNEL` constant). Change this value in **both** files if you need to avoid interference.
- **PID Tuning:** Default gains are conservative. Adjust `rollPid`, `pitchPid`, and `yawPid` values in the flight controller sketch to suit your frame and motor/prop combination.
- **Altitude Hold Placeholder:** Barometer data is filtered but not yet tied into throttle control. Use `aux1`/`aux2` or a button to implement your own hold logic.
- **LED/Buzzer:** LED is steady when armed, slow blink disarmed, fast blink + buzzer in failsafe.

Always fly in a safe, open area, follow local regulations, and keep people clear of the rotor disk.

---

## 9. Next Steps

- Add telemetry downlink (battery voltage, RSSI) via nRF24 ack payloads.
- Map buttons to buzzer toggles or arming confirmation.
- Integrate magnetometer (e.g., HMC5883L) for yaw hold and GPS for position hold.

Fly safe! If you modify pin assignments or add new features, keep the struct definitions in both sketches identical to guarantee consistent packet decoding.***
