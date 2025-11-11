## System Architecture

### Overview
- **Flight Controller (FC):** Arduino Nano (ATmega328P) managing IMU stabilization (MPU6050), ESC motor outputs, NRF24L01 radio link, buzzer, status LED, and data telemetry.
- **Radio Controller (RC):** Arduino Nano handling dual joysticks, toggle arm/kill switch, two push buttons, buzzer feedback, Nokia 5110 LCD status UI, and NRF24L01 radio link.
- **Power:** 2–4S LiPo with a PDB; 5 V BEC regulator feeds both Nanos, NRF modules, sensors, LEDs, and radio display. ESCs powered directly from PDB.

### Wiring Summary

#### Flight Controller
- **Arduino Nano VIN** ← 5 V BEC output (common ground with battery negative, ESC grounds, and RC).
- **MPU6050**: VCC → 5 V, GND → GND, SCL → A5, SDA → A4, INT → D2.
- **NRF24L01**: VCC → 3.3 V regulator (LC filter recommended), GND → GND, CE → D7, CSN → D8, SCK → D13, MOSI → D11, MISO → D12, IRQ (optional) → D6.
- **ESC Signals**: D3 → Front Right (Motor 1), D5 → Rear Right (Motor 2), D9 → Rear Left (Motor 3), D10 → Front Left (Motor 4). All ESC grounds tied to FC ground.
- **Buzzer**: D4 (with NPN transistor if buzzer >30 mA) and GND.
- **Status LED**: D6 (or any PWM pin if IRQ unused) with resistor to ground.
- **Power LED**: Optional on 5 V rail.

#### Radio Controller
- **Arduino Nano VIN** ← 5 V (from LiPo via buck converter or USB power bank).
- **NRF24L01**: VCC → 3.3 V regulator, GND, CE → D7, CSN → D8, SCK → D13, MOSI → D11, MISO → D12.
- **Nokia 5110 LCD**: VCC → 3.3 V, GND → GND, SCE → D6, RST → D5, D/C → D4, DIN → D9, CLK → D10, LED → via resistor to 3.3 V.
- **Joysticks**:
  - Left Stick (Throttle/Yaw): Vertical axis → A0 (throttle), Horizontal axis → A1 (yaw).
  - Right Stick (Pitch/Roll): Vertical axis → A2 (pitch), Horizontal axis → A3 (roll).
  - Each joystick supply: +5 V, GND. Add 0.1 µF decoupling caps to ground.
- **Toggle Switch (Arm/Kill)**: 3-position (Kill → Disarmed → Arm). Wire center pole to D2 (with INPUT_PULLUP), outer poles to GND and 5 V to create discrete states.
- **Button 1 (Calibrate)**: D3 with INPUT_PULLUP, momentary to GND.
- **Button 2 (Motor Test/Start)**: D4 with INPUT_PULLUP, momentary to GND. (Note: adjust to avoid conflict with LCD; see firmware pin map.)
- **Buzzer**: D5 via transistor to ground (optional).
- **Status LED**: D6 via resistor to ground.

### Power and Grounding
- Use a dedicated 5 V 3 A BEC for FC electronics.
- NRF24L01 requires a stable 3.3 V supply (LC filter + 47 µF capacitor near module).
- Tie all grounds together (ESCs, FC, RC, LiPo negative).
- Add TVS diodes and ferrite beads to reduce EMI from ESCs.

### Software Layers

| Layer | Flight Controller | Radio Controller |
|-------|------------------|------------------|
| Hardware Abstraction | `Wire`, `Servo`, `RF24`, Timer interrupt for IMU | `RF24`, `SPI`, `Adafruit_PCD8544` |
| Sensor Fusion | Complementary filter blending MPU6050 gyro/accel | Joystick filtering (deadband + scaling) |
| Control Loop | 500 Hz attitude loop; 250 Hz motor mixing | 100 Hz input sampling; 25 Hz UI refresh |
| Safety | Arming state machine, failsafe watchdog, ESC soft-start | Toggle arm/kill check, calibration prompts |
| Telemetry | NRF uplink to RC + buzzer/LED status | LCD wizard, haptic/LED feedback |

### Communication Protocol
- **RF Channel**: Chosen `RF24` address `0xE8E8F0F0E1LL`, 2 Mbps data rate, 32-byte payloads.
- **Packet Structure (RC → FC)**: 18 bytes
  - Header `0xA5`
  - Sequence ID (1 byte, roll-over)
  - Modes bitfield: arming, calibration request, motor test, kill, UI ack
  - Throttle (uint16_t, 1000–2000 µs)
  - Roll, Pitch, Yaw commands (int16_t, ±500 units)
  - Auxiliary (button states)
  - CRC8 (Dallas/Maxim polynomial)
- **Packet Structure (FC → RC)**: 18 bytes
  - Header `0x5A`
  - Sequence ID echo
  - Status bitfield: armed, calibrated, failsafe, imu healthy
  - Battery voltage (uint16_t, mV)
  - Altitude (int16_t, cm) from barometer placeholder
  - Roll/Pitch/Yaw actual (int16_t, 0.01° units)
  - Motor outputs (uint16_t average)
  - CRC8.

### Control Algorithm
- **Sensor Update**: 1 kHz raw gyro/acc read from MPU6050; complementary filter at 500 Hz to compute Euler angles.
- **PID Controllers**:
  - Outer loop: Stabilize attitude by comparing RC setpoints to filtered angles.
  - Inner loop: Rate PID on gyro data to damp fast rotations.
- **Motor Mixer** (X-frame):
  - `M1 = throttle + pitch + yaw`
  - `M2 = throttle - roll - yaw`
  - `M3 = throttle - pitch + yaw`
  - `M4 = throttle + roll - yaw`
  - Clamp 1000–2000 µs; map to ESC command.
- **Throttle Safety**: RC calibrates joystick ranges; throttle mapped from 0–100% -> 1100–1900 µs; 1000 µs only in disarmed mode to avoid accidental spin-up.

### Calibration & Safety Sequence
1. **Power On**: RC boots, waits for NRF link. Displays “Searching...” on Nokia 5110. FC buzzer chirps when link established.
2. **Kill Switch Check**: RC ensures toggle in Kill. If not, buzzer + LCD prompt.
3. **IMU/ESC Calibration**:
   - Press Button 1. RC sends calibration request; FC performs MPU6050 bias estimation (8 s) and ESC min/max capture.
   - FC stores offsets in EEPROM and responds with status. RC shows progress and success/failure.
4. **Arm Sequence**:
   - Toggle to Arm position. RC verifies throttle <10%, yaw centered; FC enters “armed but motors idle”.
   - Press Button 2 to soft-start motors (gradual ramp to 10% over 3 s).
5. **Flight**: Control with joysticks. Display shows real-time data: throttle, roll, pitch, yaw, altitude, RF channel, battery.
6. **Disarm**: Toggle to Kill or hold Button 2 for 2 s. FC cuts motors, resets failsafe, logs event.

### Failsafe Strategies
- RF watchdog: if no radio packet for >250 ms, FC enters failsafe (buzzer alarm, LED flash, motors to 1000 µs).
- Tilt limit: If absolute roll/pitch >75°, FC auto-disarms.
- Battery monitor: <3.5 V per cell triggers low battery warning on LCD + buzzer chirps.
- ESC soft-start and slew rate limit to reduce sudden commands.

### Expansion Hooks
- Add BMP280 for real altitude.
- Log data to micro SD.
- GPS waypoint support via serial.
- Telemetry override to smartphone via HC-05 Bluetooth.
