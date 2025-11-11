# Quadcopter Flight Controller & RC Ground Station (Arduino Nano)

## Overview

This project implements a complete DIY quadcopter stack made of two Arduino Nano boards communicating over NRF24L01 radios:

- **Flight Controller (FC)**: Reads an MPU6050 IMU, stabilises the aircraft with PID control, mixes motor outputs for four ESCs, drives a buzzer and a status LED, and reports telemetry back to the transmitter.
- **RC Transmitter (RC)**: Reads two joysticks, a two-position arm/kill toggle, two push buttons, and renders user guidance plus live telemetry on a Nokia 5110 display. It manages stick calibration so the quad never arms with unsafe throttle values.

The firmware enforces a guided start-up checklist (radio link → kill switch confirmation → calibration → arming → smooth idle spin) and contains failsafes that disarm and alert whenever the link drops.

## Hardware

### Core Components

| Subsystem | Component |
|-----------|-----------|
| Flight Controller | Arduino Nano, MPU6050 IMU, NRF24L01, 4× ESC + brushless motor, active buzzer, status LED, LiPo voltage divider |
| RC Transmitter | Arduino Nano, NRF24L01, 2× 2-axis joysticks, 2× push buttons, 1× toggle switch (arm/kill), Nokia 5110 display (PCD8544), status LED |

### Flight Controller Wiring

| Signal | Arduino Nano Pin | Notes |
|--------|------------------|-------|
| ESC Front Left | D3 (PWM) | `Servo` library outputs |
| ESC Front Right | D5 (PWM) | |
| ESC Rear Left | D6 (PWM) | |
| ESC Rear Right | D9 (PWM) | |
| NRF24L01 CE | D7 | |
| NRF24L01 CSN | D8 | Hardware SPI on D11 (MOSI), D12 (MISO), D13 (SCK) |
| MPU6050 SDA | A4 | I²C |
| MPU6050 SCL | A5 | I²C |
| Buzzer (+) | D4 | Use small NPN transistor for >20 mA buzzer |
| Status LED (+) | D13 | On-board LED is acceptable |
| Battery sense (divider mid-point) | A7 | Configure divider for ≤5 V input |
| LiPo GND | GND | Common ground required |

Power the FC with a regulated 5 V BEC capable of feeding the Nano, radio module, and sensors. ESC power rails must share ground with the Nano.

### RC Transmitter Wiring

| Signal | Arduino Nano Pin | Notes |
|--------|------------------|-------|
| NRF24L01 CE | D9 | |
| NRF24L01 CSN | D10 | |
| Joystick THR (vertical left) | A0 | Use 3.3–5 V reference |
| Joystick YAW (horizontal left) | A1 | |
| Joystick PITCH (vertical right) | A2 | |
| Joystick ROLL (horizontal right) | A3 | |
| Button 1 (Calibrate) | D2 | `INPUT_PULLUP` |
| Button 2 (Smooth idle spin) | D3 | `INPUT_PULLUP` |
| Arm/Kill toggle | D8 | `INPUT_PULLUP` — connect kill position to GND |
| Status LED | D4 | Series resistor required |
| Nokia 5110 RST | D5 | |
| Nokia 5110 DC | D6 | |
| Nokia 5110 CS | D7 | Uses hardware SPI (MOSI=D11, SCK=D13) |
| RC battery monitor (optional) | A6 | Adjust divider for ≤5 V |

Both NRF24L01 modules benefit greatly from a 10 µF–47 µF decoupling capacitor across VCC/GND and short leads.

## Firmware Layout

```
firmware/
├── flight_controller/
│   └── flight_controller.ino
├── rc_transmitter/
│   └── rc_transmitter.ino
└── shared/
    └── comm_protocol.h
```

- `comm_protocol.h` defines the RF packet layout, CRC helpers, and shared enums so the FC and RC stay in sync.
- `flight_controller.ino` contains the IMU readout, complementary filter, PID loops, motor mixing, calibration routines, buzzer/LED patterns, telemetry, and state machine enforcing the guided workflow.
- `rc_transmitter.ino` implements stick calibration stored in EEPROM, joystick mapping with deadbands, the guided checklist UI on the Nokia display, action button handling, and telemetry visualisation.

## Required Arduino Libraries

Install the following (through the Arduino Library Manager or manually):

- **RF24** by TMRh20 (`<RF24.h>`)
- **I2Cdevlib MPU6050** (`<MPU6050.h>`)
- **Servo** (bundled with Arduino IDE)
- **Adafruit PCD8544 Nokia 5110 LCD** (`<Adafruit_PCD8544.h>`)

Be sure to select **Arduino Nano** (Optiboot/Atmega328P) in the IDE for both sketches.

## Building & Uploading

1. Open `firmware/flight_controller/flight_controller.ino` in the Arduino IDE, select the FC Nano’s COM port, verify, and upload.
2. Open `firmware/rc_transmitter/rc_transmitter.ino`, select the transmitter Nano’s port, verify, and upload.
3. Power-cycle both devices after flashing so EEPROM reads initialise correctly.

## Guided Start-Up Flow (Transmitter Display)

1. **NRF link...** – RC searches for the FC; buzzer chirps once a connection is up.
2. **Set toggle KILL** – Confirm the arm/kill switch is in kill position; FC won’t proceed otherwise.
3. **Press BTN1 CAL** – Press Button 1. RC performs stick calibration, then commands the FC to calibrate IMU + ESCs. Buzzer + LED indicate progress.
4. **Calibrating…** – FC runs IMU and ESC calibration; motors stay disarmed.
5. **Flip toggle ARM** – Toggle to arm position while keeping throttle down. FC arms when safe.
6. **Press BTN2 Spin** – Button 2 requests a gentle motor idle spin (no lift-off).
7. **Ready to fly!** – Live telemetry (throttle, roll, pitch, yaw, estimated altitude, battery) remains on-screen.

At any point, switching back to kill instantly disarms and stops the motors.

## Controls

- **Left joystick (vertical)**: Throttle (1000–2000 µs). Calibration guarantees the bottom position maps to 1000 and prevents accidental centre=1000 issues.
- **Left joystick (horizontal)**: Yaw rate command.
- **Right joystick (vertical)**: Pitch.
- **Right joystick (horizontal)**: Roll.
- **Button 1**: Runs RC stick calibration (stored to EEPROM) and triggers FC IMU+ESC calibration. Use whenever hardware changes or drift appears.
- **Button 2**: Commands a smooth idle spin for motor validation.
- **Toggle**: Kill (down/LOW) vs. Arm/Safe (up/HIGH). The FC refuses to arm unless throttle is at minimum and calibrations completed.

## Flight Controller Features

- 400 Hz complementary filter using MPU6050 gyro/accel data.
- PID stabilisation with default gains (Roll/Pitch 4.0/0.04/16.0, Yaw 3.0/0.03/0.0). Adjust in code as needed.
- Motor mixing for an X-quad layout (front-left, front-right, rear-left, rear-right).
- Failsafe: if no valid control frame arrives for >150 ms, the FC disarms, silences motors, sets telemetry flags, and awaits link recovery.
- Buzzer and LED patterns for key events (link established, calibration complete, armed/disarmed, failsafe).
- Estimated altitude from vertical acceleration (complementary filtered). For precise altitude hold consider adding a barometer.

## RF Link & Telemetry

- Packets are 32 bytes with CRC16 to guard against corruption.
- Control frames carry throttle + roll/pitch/yaw offsets, arm state, and momentary action flags.
- Telemetry frames return attitude (deg ×100), throttle echo, estimated altitude, battery voltage, status flags, and the current checklist phase.
- NRF24L01 is configured at channel 92 (2.492 GHz), 1 Mbps, high power. Adjust channel if interference occurs.

## Calibration Details

### RC Stick Calibration

1. Ensure toggle is in **kill**.
2. Press **Button 1**.
3. Follow screen instructions: move sticks to all extremes for 5 s, then release to centre.
4. Calibration data writes to EEPROM and survives power cycles.
5. Throttle is forced to 1000 µs while calibrating to keep the FC safe.

### FC IMU & ESC Calibration

Triggered automatically after RC calibration completes (Button 1). Keep the quad absolutely still and make sure the ESCs are connected and powered. The FC sends the classical high/low PWM sequence to each ESC and stores IMU offsets in EEPROM.

## Safety Notes

- Always mount the quad securely when first testing. Validate each motor spins in the correct direction before installing props.
- Use a proper LiPo voltage divider with known ratio; update the code if your divider differs.
- The FC requires a 5 V supply stable under load (NRF24L01 is sensitive to noise).
- Keep NRF antennas away from power wiring and ESCs.
- If you modify PID gains, test with props removed first.

## Extending the System

- Integrate a barometer (BMP280/BMP388) for accurate altitude and feed its readings into `TelemetryFrame::altitude_cm`.
- Add SD logging via SPI (ensure unique CS pins).
- Implement arming logic that requires throttle low + yaw hold if you prefer stick arming instead of a toggle.
- Tune PID values, loop rate, and motor mixing for your specific frame geometry.

## Troubleshooting

- **No NRF link**: Check 3.3 V supply to NRF24L01, wiring (CE/CSN), and verify channel address matches both sketches.
- **No telemetry updates**: Ensure `radio.enableAckPayload()` succeeds on both sides; confirm the FC is running and not stuck in failsafe.
- **Motors won’t arm**: Confirm calibrations completed (Button 1), throttle is below 1020 µs, and toggle is in arm position.
- **Display blank**: Verify contrast setting, backlight power, and wiring to RST/DC/CS.

Once the guided checklist completes and the telemetry shows “Ready to fly!”, the quad responds to stick inputs smoothly and predictably. Happy flying! 🚁
