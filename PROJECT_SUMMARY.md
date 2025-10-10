# Arduino Flight Controller - Fixed and Optimized

## Overview

This is a complete Arduino-based quadcopter flight controller with the following features:
- **Attitude stabilization** using MPU6050 IMU (gyroscope + accelerometer)
- **Altitude hold** using MS5611 barometer with Kalman filtering
- **Radio control** via nRF24L01 2.4GHz transceiver
- **PID control loops** for roll, pitch, yaw, and altitude
- **Safety features** including failsafe, angle limits, and arming sequence

## Critical Fixes Applied

### 🔴 **CRITICAL: Kalman Filter Bug Fixed**
The original code called `initKalmanPosVel()` every loop iteration, completely breaking the filter by resetting the covariance matrix. This has been fixed - initialization now happens only once in `setup()`.

### ⚠️ **17 Total Issues Fixed**
See `FIXES_AND_OPTIMIZATIONS.md` for complete details.

## Project Structure

```
workspace/
├── FlightController/              # Main flight controller (on quadcopter)
│   ├── FlightController.ino       # Main code
│   ├── Gyro.cpp                   # MPU6050 driver implementation
│   ├── Gyro.h                     # MPU6050 driver header
│   └── libraries/
│       └── Smoothed/              # Sensor smoothing library
│           ├── Smoothed.h
│           └── library.properties
│
├── Controller/                    # RC transmitter code
│   └── Controller.ino
│
├── FIXES_AND_OPTIMIZATIONS.md     # Detailed changelog
└── PROJECT_SUMMARY.md             # This file
```

## Hardware Requirements

### Flight Controller Board
- **Microcontroller**: Arduino Nano/Uno (ATmega328P)
- **IMU**: MPU6050 (gyroscope + accelerometer)
- **Barometer**: MS5611 (altitude sensing)
- **Radio**: nRF24L01 (2.4 GHz transceiver)
- **ESCs**: 4x compatible with 1000-2000μs PWM
- **Battery**: 3S-4S LiPo (voltage divider for monitoring)
- **Buzzer**: For status alerts
- **LED**: For visual status

### Pin Connections - Flight Controller

| Component | Pin | Notes |
|-----------|-----|-------|
| ESC Front-Left | D3 | PWM output |
| ESC Front-Right | D5 | PWM output |
| ESC Rear-Right | D6 | PWM output |
| ESC Rear-Left | D9 | PWM output |
| nRF24L01 CE | D4 | |
| nRF24L01 CSN | D10 | |
| Buzzer | D8 | Active buzzer |
| LED | D7 | Status indicator |
| MPU6050 SDA | A4 | I2C data |
| MPU6050 SCL | A5 | I2C clock |
| MS5611 SDA | A4 | I2C data (shared) |
| MS5611 SCL | A5 | I2C clock (shared) |
| Battery Monitor | A0 | Via voltage divider |

### RC Controller Board
- **Microcontroller**: Arduino Nano/Uno
- **Radio**: nRF24L01
- **Joysticks**: 2x dual-axis analog joysticks
- **Buttons**: 2x push buttons
- **Switches**: 2x toggle switches

### Pin Connections - Controller

| Component | Pin | Notes |
|-----------|-----|-------|
| nRF24L01 CE | D9 | |
| nRF24L01 CSN | D10 | |
| Joystick Y-Left | A0 | Thrust |
| Joystick X-Left | A1 | Yaw |
| Joystick Y-Right | A2 | Pitch |
| Joystick X-Right | A3 | Roll |
| Button 1 | D4 | Calibration |
| Button 2 | D5 | Arm/Disarm |
| Switch 1 | D3 | Kill switch |
| Switch 2 | D2 | Altitude hold |

## Software Dependencies

Install these libraries via Arduino Library Manager:
1. **RF24** - nRF24L01 radio library
2. **MS5611** - Barometer library
3. **Smoothed** - Included in project
4. Standard libraries (Wire, SPI, Servo, EEPROM)

## Key Features

### 1. PID Attitude Control
- **Roll/Pitch**: P=2.0, I=0.0001, D=0.5
- **Yaw**: P=2.0
- Anti-windup protection on integral terms
- Complementary filter (99% gyro, 1% accelerometer)

### 2. Altitude Hold
- Kalman filter for sensor fusion
- Adaptive P-gain based on error magnitude
- Barometer-based altitude estimation
- Vertical velocity damping

### 3. Safety Features
- **Arming sequence**: Hold button 2 for 2 seconds
- **Kill switch**: Switch 1 disarms immediately
- **Angle limit**: 180° max tilt (configurable)
- **Failsafe**: Auto-disarm after 3 seconds signal loss
- **Pre-flight calibration**: Hold button 1 for 2 seconds

### 4. Flight Modes
- **Manual mode**: Direct throttle control
- **Altitude hold**: Activate with switch 2 (thrust 1400-1450)

## Configuration Parameters

### Flight Controller Settings

```cpp
// PID Tuning
const float kp = 2.0;        // Proportional gain
const float ki = 0.0001;     // Integral gain
const float kd = 0.5;        // Derivative gain
const float kpZ = 2.0;       // Yaw P gain

// Altitude PID
float pid_p_gain_altitude = 14.0;
float pid_i_gain_altitude = 2.0;
float pid_d_gain_altitude = 7.5;

// Sensitivity
float sensiX = -0.45;        // Roll sensitivity
float sensiY = 0.45;         // Pitch sensitivity
float sensiZ = -0.01;        // Yaw sensitivity
float sensiThrust = 1.1;     // Throttle scaling

// Motor Limits
int pMAX = 2000;             // Max PWM
int pMIN = 1000;             // Min PWM
int MINarmed = 1050;         // Armed min (keeps motors spinning)
int maxThrust = 1700;        // Max thrust limit

// Safety
int maxAngle = 180;          // Max tilt angle (degrees)
bool killAngle = true;       // Enable angle limit
float hz = 140.0;            // Loop frequency
```

### Radio Settings (MUST MATCH on both boards)

```cpp
const uint64_t pipe = 0xF0F0F0F0E1LL;  // Pipe address
radio.setChannel(35);                   // Channel 35 = 2.435 GHz
radio.setDataRate(RF24_250KBPS);       // Data rate
radio.setPALevel(RF24_PA_LOW);          // Power level
```

## Pre-Flight Checklist

### ⚠️ SAFETY FIRST - Remove propellers for all tests!

1. **Hardware Setup**
   - [ ] All connections secure
   - [ ] Battery fully charged
   - [ ] Propellers REMOVED
   - [ ] ESCs calibrated (if needed)

2. **Software Upload**
   - [ ] Upload FlightController.ino to quadcopter
   - [ ] Upload Controller.ino to transmitter
   - [ ] Verify serial output shows "Setup complete"

3. **Calibration**
   - [ ] Place quadcopter on level surface
   - [ ] Power on and wait for beeps
   - [ ] Hold button 1 for 2+ seconds (calibration)
   - [ ] Wait for confirmation beeps

4. **Radio Test**
   - [ ] Power on controller first
   - [ ] Power on flight controller
   - [ ] Verify LED/serial shows radio connection
   - [ ] Move sticks - motors should NOT spin (not armed)

5. **Arming Test**
   - [ ] Hold button 2 for 2+ seconds
   - [ ] Confirm beep and motors spin at low speed
   - [ ] Move sticks - verify correct motor responses
   - [ ] Test kill switch (switch 1)

6. **Motor Direction Test** (Props still OFF!)
   - [ ] Forward stick → front motors speed up
   - [ ] Backward stick → rear motors speed up
   - [ ] Right stick → right motors speed up
   - [ ] Left stick → left motors speed up
   - [ ] Yaw right → CW motors speed up
   - [ ] Yaw left → CCW motors speed up

7. **First Flight** (Add propellers now)
   - [ ] Start in open area
   - [ ] Have observer for safety
   - [ ] Test hover at low altitude first
   - [ ] Tune PID gains if oscillation occurs
   - [ ] Monitor battery voltage

## PID Tuning Guide

### If quadcopter oscillates:
1. **Reduce P gain** by 20-30%
2. Set I gain to 0 initially
3. Increase D gain slightly for damping

### If quadcopter drifts:
1. **Increase I gain** slowly (0.0001 → 0.001)
2. Recalibrate gyro on level surface

### If response is sluggish:
1. **Increase P gain** by 10-20%
2. Ensure D gain is not too high

### Recommended tuning sequence:
1. Start with P only (I=0, D=0)
2. Increase P until slight oscillation
3. Reduce P by 30%
4. Add D for damping
5. Add small I to eliminate steady-state error

## Serial Debug Output

Enable debugging by setting:
```cpp
debugging(true);  // In setup()
```

Output includes:
- Barometric pressure (raw and filtered)
- Kalman filter height estimate
- Vertical velocity
- PID errors
- Motor outputs

Baud rate: **57600**

## Troubleshooting

### Motors don't spin when armed
- Check ESC connections
- Verify ESC calibration
- Check MIN/MAX PWM values match your ESCs
- Verify power supply is adequate

### Quadcopter flips on takeoff
- Motor direction incorrect - check props/motor rotation
- Check motor mixing in `calculateVelocities()`
- Verify gyro orientation matches code

### Radio disconnects
- Check antenna on nRF24L01
- Reduce distance for testing
- Add capacitor (10μF) to nRF24L01 power
- Verify matching channel and pipe

### Altitude hold doesn't work
- Calibrate barometer at ground level
- Check MS5611 I2C connection
- Verify Kalman filter is updating (debug output)
- Ensure switch 2 activates at correct throttle range

### Gyro drift
- Recalibrate on perfectly level surface
- Increase complementary filter acc weight (0.01 → 0.02)
- Check for vibration isolation

## Advanced Modifications

### Change loop frequency
```cpp
float hz = 200.0;  // Change to desired frequency
// Also update TIMESLICE in Kalman filter:
#define TIMESLICE 0.005  // 1/200
```

### Add GPS
- Include GPS library
- Add position hold PID loops
- Implement waypoint navigation

### Add telemetry
- Use second nRF24L01 for downlink
- Transmit battery, altitude, position
- Receive on ground station

### Implement acro mode
- Disable angle stabilization
- Control angular rates directly
- Add rate limiting for safety

## License & Credits

Original code structure from various open-source flight controller projects.
Fixes and optimizations applied 2025.

**⚠️ DISCLAIMER**: This is experimental software for educational purposes. Fly at your own risk. Always follow local regulations for UAV operation.

## Contributing

Found a bug or have an improvement? Contributions welcome:
1. Test thoroughly
2. Document changes
3. Follow existing code style
4. Update documentation

## Version History

- **v2.0** - Fixed critical bugs, added optimizations (current)
  - Fixed Kalman filter initialization bug
  - Added integral windup protection
  - Improved safety checks
  - Optimized floating-point precision
  
- **v1.0** - Original code (had critical bugs)

---

**Happy Flying! 🚁**

Remember: Safety first, test incrementally, and always have a backup plan (like a kill switch).
