## Quadcopter Flight Controller & RC System Overview

### 1. Bill of Materials
- **Flight Controller**
  - Arduino Nano (ATmega328P, 16 MHz)
  - MPU6050 6-DoF IMU
  - NRF24L01+ 2.4 GHz transceiver (with PA+LNA module recommended)
  - 4 x ESCs (SimonK/BLHeli with 1-2 kHz PWM input, 20-30 A depending on motors)
  - 4 x Brushless DC motors + props
  - Power distribution board (PDB) or wiring harness
  - LiPo battery (3S-4S 1500-2200 mAh typical)
  - 5 V BEC (2 A minimum) to power logic, NRF, sensors, buzzer, LEDs
  - Active buzzer (5 V)
  - Flight status LED (green) + series resistor (220 ohm)
  - Warning LED (red) + series resistor (220 ohm) - optional
  - Capacitance (100 uF electrolytic) close to NRF24L01+ VCC
- **RC Transmitter**
  - Arduino Nano
  - NRF24L01+
  - 2 x 2-axis analog joysticks (10 kohm pots)
  - Toggle switch (SPDT locking, used for Arm/Kill)
  - 2 x momentary push-buttons (NO) for Calibrate and Motor-Spin tests
  - Nokia 5110 LCD (PCD8544 controller)
  - LiPo/Li-ion power source (2S pack or 7.4 V) + 5 V regulator (1 A)
  - Status LED (bi-color preferred, else single color) + resistor
  - Buzzer (5 V piezo) - optional for RC feedback

### 2. Wiring Summary

#### 2.1 Flight Controller
| Component | Arduino Pin | Notes |
|-----------|-------------|-------|
| MPU6050 SDA | A4 | Requires `Wire` pull-ups (IMU board includes) |
| MPU6050 SCL | A5 | |
| MPU6050 INT | D2 | External interrupt for new data |
| NRF24L01+ CE | D7 | |
| NRF24L01+ CSN | D8 | |
| NRF24L01+ SCK | D13 | Hardware SPI |
| NRF24L01+ MISO | D12 | |
| NRF24L01+ MOSI | D11 | |
  | NRF24L01+ VCC | 3.3 V | Use separate AMS1117 regulator or BEC; add 100 uF cap |
| NRF24L01+ GND | GND | Star-ground with IMU |
| ESC 1 (Front Left) | D3 | Timer2 PWM ~490 Hz (mapped to motor front-left) |
| ESC 2 (Front Right) | D5 | Timer0 PWM ~980 Hz |
| ESC 3 (Rear Right) | D6 | Timer0 PWM |
| ESC 4 (Rear Left) | D9 | Timer1 PWM |
| Buzzer + | D4 | Active buzzer driven HIGH when link OK |
| Status LED | D10 | Blink on telecom activity |
| Battery voltage divider | A0 | Optional for low-voltage alarm |
| Altitude sensor (optional BMP280) SDA | A4 | Shared I2C |
| Altitude sensor SCL | A5 | |

All ESC grounds connect to Arduino GND. Route ESC BEC (if present) to power Nano only if dedicated BEC not used (avoid powering from multiple ESC BECs simultaneously).

#### 2.2 RC Transmitter
| Component | Arduino Pin | Notes |
|-----------|-------------|-------|
| Joystick 1 X (Yaw) | A0 | Input 0-5 V |
| Joystick 1 Y (Throttle) | A1 | Use mechanical centering spring removal for throttle if desired |
| Joystick 2 X (Roll) | A2 | |
| Joystick 2 Y (Pitch) | A3 | |
| Toggle switch Arm/Kill | D2 | Use pull-down resistor, or enable `INPUT_PULLUP` with inverted logic |
| Button 1 (Calibrate) | D3 | `INPUT_PULLUP` |
| Button 2 (Motor Spin) | D4 | `INPUT_PULLUP` |
| Nokia 5110 SCLK | D13 | Shared SPI |
| Nokia 5110 DIN | D11 | |
| Nokia 5110 D/C | D9 | |
| Nokia 5110 CS | D10 | |
| Nokia 5110 RST | D8 | |
| Backlight | D5 | PWM for dimming |
| NRF24L01+ CE | D6 | |
| NRF24L01+ CSN | D7 | |
| NRF24L01+ SCK | D13 | Shared with LCD |
| NRF24L01+ MOSI | D11 | Shared |
| NRF24L01+ MISO | D12 | |
| Status LED | D14/A0 (use digital pin via `digitalPinToInterrupt`) | Or reuse remaining digital I/O |
| Buzzer | D15/A1 | Optional feedback |

Provide regulated 3.3 V to NRF module with decoupling. LCD runs at 3.3 V-level shifters recommended if powering logic at 5 V; they tolerate 5 V inputs but safer to reduce to 3.3 V (use resistor dividers on D/C, DIN, SCLK, CS).

### 3. Power & Safety
- Use a dedicated 5 V BEC to power Nano, peripherals, and NRF. Keep grounds common.
- Add TVS diode and LC filter on LiPo input to reduce ESC switching noise.
- Set Arduino Nano `VIN` when feeding from 7-12 V regulator; prefer 5 V through `5V` pin for minimal heat.
- Install blade guards and keep props off during bench testing.
- Implement Arm/Kill solenoid: ESC outputs forced to 1000 us when disarmed, 900 us when kill asserted.

### 4. Software Architecture Overview
1. **Flight Controller**
   - `TaskScheduler` style loop at 2 kHz IMU read, 500 Hz control update, 250 Hz radio decode, 50 Hz telemetry.
   - Complementary filter for attitude (gyro + accelerometer).
   - PID controllers (roll, pitch, yaw rate).
   - State machine: `POWER_ON` → `LINK_WAIT` → `SAFE_KILL` → `CALIBRATION` → `ARMED` → `MOTOR_SPIN` → `FLIGHT`.
   - Calibration command stores offsets and ESC min/max to EEPROM.
   - NRF payload includes RC commands + metadata; returns telemetry to RC for LCD.
2. **RC Transmitter**
   - Startup wizard guiding the operator via Nokia 5110.
   - Joystick auto-calibration (min/center/max) stored in EEPROM per axis.
   - Safety gating: throttle forced to floor until arm confirmed and Motor Spin button pressed.
   - Provides explicit command frames with CRC and sequence numbers.
   - Displays live telemetry (Throttle %, Roll/Pitch/Yaw deg, estimated altitude, link RSSI).

### 5. Communication Protocol (NRF24L01+)
- Channel: 110 (2.510 GHz), Data rate: 1 Mbps (robust), Payload size: 32 bytes.
 - Packet Structure (TX -> FC):
  ```
  Byte 0: Header (bitfield: armed, kill, calibrate, motor_spin, seq)
  Byte 1: Throttle low byte (0-1000)
  Byte 2: Throttle high byte
  Byte 3: Roll command (signed int16, +/-500)
  Byte 5: Pitch command (signed int16, +/-500)
  Byte 7: Yaw rate command (signed int16, +/-500)
  Byte 9: Toggle state, button states, reserved
  Byte 10-11: CRC16
  ```
  - FC -> TX telemetry: attitude (roll/pitch/yaw), altitude estimate, battery voltage, link OK flag, arming state, error codes.
- Use Auto-Acknowledgment with 3 retries; buzzer + LED confirm link.

### 6. Calibration Flow
1. **Joystick Calibration (on RC)**: On first boot or when requested in menu, sweep sticks to extremes; auto-save min/center/max to EEPROM.
2. **IMU Calibration (Button 1)**: RC sends command; FC averages 5 seconds of gyro/accel data while stationary, stores offsets.
3. **ESC Calibration**: FC commands max/min pulses in sequence; instruct user to connect LiPo during calibration.
4. **Level Trim**: After IMU calibration, optional small adjustments stored per-axis.

### 7. Safety Interlocks
- Kill switch overrides any command; FC drives ESC outputs to 1000 us.
- Arming only allowed when throttle < 5 %, kill switch disengaged, calibration flag valid, and IMU stable.
- Motor Spin button engages slow ramp from min throttle to idle (1150 us) so that props begin spinning uniformly without lift.
- Link loss -> FC enters failsafe: throttle to 0, buzzer pulses, LED flashes SOS.
- RC monitors heartbeat; if ack lost for > 500 ms, LCD warns operator and forces control outputs to safe defaults.

### 8. Estimated Altitude
- MPU6050 accelerometer cannot provide absolute altitude; integrate vertical acceleration with aggressive filtering for short-term hover.
  - Recommend integrating BMP280 or VL53L1X for reliable altitude. If unavailable, present altitude as "Est Alt (relative)" with zeroed value at arm time, drift-corrected via complementary filter (accelerometer vs. throttle-based model).

### 9. Mechanical Considerations
- Mount MPU6050 on vibration-damping foam, oriented so axes align with frame.
- Balance props, secure wiring away from prop arc.
- Ensure RF antennas (both sides) vertical and unobstructed; use PA/LNA versions for >200 m range.

### 10. Development & Test Plan
1. Bench-test RC link without props; verify LCD prompts and command flow.
2. Calibrate IMU/ESC with frame grounded.
3. Attach props, run Motor Spin test in a tethered rig.
4. Conduct first hover indoors with protective net or outdoors low altitude.
5. Tune PID gains incrementally: start conservative (P only), then add D, finally small I.

Keep props removed until firmware validated.
