# Arduino Nano Quadcopter & RC System

This workspace contains a complete Arduino Nano-based quadcopter flight controller and companion RC transmitter firmware. It implements:
- MPU6050-based attitude estimation with complementary filtering
- PID-stabilised motor mixing for an X-frame quad (four ESC outputs)
- NRF24L01+ bidirectional telemetry link
- Guided arming/calibration workflow using a Nokia 5110 display
- Joystick calibration that fixes the throttle-centre safety issue (min->1000 us, max->2000 us)
- Audible/status LED feedback for link, calibration, and failsafe events

## Repository Layout
- `docs/quadcopter_system_overview.md` - hardware BOM, wiring, control architecture, safety notes
- `docs/bringup_and_operation.md` - flashing, calibration, and operating procedures
- `firmware/flight_controller/flight_controller.ino` - aircraft firmware
- `firmware/radio_controller/radio_controller.ino` - handheld transmitter firmware

## Quick Start
1. Install required Arduino libraries (`RF24`, `Adafruit_PCD8544`, `Adafruit_GFX`, `MPU6050`).
2. Wire the flight controller and transmitter per the overview document.
3. Upload the corresponding firmware sketches to each Arduino Nano.
4. Follow the bring-up guide for joystick, IMU, and ESC calibration, then proceed through the on-screen checklist before attempting flight.

Always remove propellers during calibration and bench tests, and verify throttle is at absolute minimum before arming. Refer to the documentation under `docs/` for detailed guidance and troubleshooting.*** End Patch
