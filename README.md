# Quadcopter Flight Controller & RC System (Arduino Nano)

This project implements a complete DIY quadcopter platform with:
- **Flight Controller**: Arduino Nano + MPU6050 IMU + NRF24L01 radio + 4× ESC outputs + buzzer + status LED.
- **Radio Controller**: Arduino Nano + dual joysticks + toggle/kill switch + two push buttons + buzzer + status LED + Nokia 5110 display.

Both firmwares exchange telemetry/control data via NRF24L01 using a robust packet format with CRC, arming states, and failsafe behaviour. The transmitter guides the pilot through calibration → arming → soft-start → flight with on-screen prompts.

## Repository Layout
- `firmware/flight_controller/flight_controller.ino` – Stabilisation loop (500 Hz), complementary filter, dual-stage PID, ESC mixing, calibration storage, radio failsafe.
- `firmware/rc_transmitter/rc_transmitter.ino` – Joystick calibration & scaling, UI wizard on Nokia 5110, radio uplink, telemetry dashboard.
- `docs/system_architecture.md` – Hardware wiring, power design, communication protocol, control algorithm details.
- `docs/operation_guide.md` – Step-by-step setup, calibration workflow, safety sequence, tuning tips.

## Quick Start
1. Install required Arduino libraries (see `docs/operation_guide.md`).
2. Upload each firmware to an Arduino Nano (select *ATmega328P (Old Bootloader)* if needed).
3. Assemble hardware per `docs/system_architecture.md`.
4. Follow the LCD wizard after power-on: Kill → Calibrate → Arm → Soft-start → Fly.

> ⚠️ Always remove propellers during calibration and bench testing. Ensure a clean 3.3 V supply for NRF24L01 modules and common grounds between ESCs and controller.
