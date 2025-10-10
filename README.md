# Arduino Quadcopter Flight Controller - Fixed & Optimized

## 🔴 CRITICAL FIXES APPLIED

This is a **fixed and optimized** version of an Arduino-based quadcopter flight controller. The original code contained several critical bugs that have been resolved.

### Major Issues Fixed:
1. ✅ **Kalman Filter Bug** (CRITICAL) - Filter was being reset every loop iteration
2. ✅ **PID Integral Windup** - No bounds on integral accumulation
3. ✅ **Division by Zero** - Missing safety checks in multiple locations
4. ✅ **Function Naming Errors** - Inconsistent capitalization causing compilation issues
5. ✅ **Altitude Hold Logic** - Redundant conditions and missing reset logic
6. ✅ **Accelerometer Filtering** - Improved complementary filter with bounds checking
7. ✅ **And 11 more fixes...** (see `FIXES_AND_OPTIMIZATIONS.md`)

## Quick Start

### 📁 Project Structure
```
FlightController/              # Main quadcopter code (upload to flight controller)
  ├── FlightController.ino
  ├── Gyro.cpp
  ├── Gyro.h
  └── libraries/Smoothed/

Controller/                    # RC transmitter code
  └── Controller.ino

FIXES_AND_OPTIMIZATIONS.md     # Detailed changelog of all fixes
PROJECT_SUMMARY.md             # Complete documentation & setup guide
```

### 🛠️ Hardware Requirements
**Flight Controller:**
- Arduino Nano/Uno (ATmega328P)
- MPU6050 IMU (gyroscope + accelerometer)
- MS5611 Barometer
- nRF24L01 Radio module
- 4× ESCs + Motors
- LiPo Battery (3S-4S)
- Buzzer & LED

**RC Transmitter:**
- Arduino Nano/Uno
- nRF24L01 Radio module
- 2× Dual-axis joysticks
- 2× Buttons, 2× Switches

### 📡 Installation Steps
1. **Install Arduino IDE** and required libraries:
   - RF24 (nRF24L01 radio)
   - MS5611 (barometer)
   - Smoothed (included)

2. **Upload Firmware:**
   - Open `FlightController/FlightController.ino` in Arduino IDE
   - Select "Arduino Nano" as board
   - Upload to flight controller
   - Open `Controller/Controller.ino` 
   - Upload to transmitter

3. **Hardware Setup:**
   - Connect all components per pin diagram in `PROJECT_SUMMARY.md`
   - **REMOVE PROPELLERS** for initial testing
   - Connect battery

4. **Calibration:**
   - Place quadcopter on level surface
   - Power on (wait for startup beeps)
   - Hold button 1 for 2+ seconds (gyro calibration)
   - Wait for confirmation beeps

5. **Test:**
   - Power on transmitter first
   - Power on flight controller
   - Verify radio connection (check serial output)
   - Arm with button 2 (hold 2 seconds)
   - Test controls (PROPS OFF!)

### ⚠️ SAFETY WARNINGS

**CRITICAL SAFETY RULES:**
- ⚠️ **ALWAYS remove propellers** for initial tests
- ⚠️ **Test in open area** away from people and property
- ⚠️ **Have a safety observer** during test flights
- ⚠️ **Know your kill switch** (switch 1 disarms immediately)
- ⚠️ **Start with low PID gains** and tune gradually
- ⚠️ **Monitor battery voltage** during flight
- ⚠️ **Follow local regulations** for UAV operation

## Features

### Flight Control
✅ **Attitude Stabilization** - Roll, pitch, yaw control with PID loops  
✅ **Altitude Hold** - Barometric altitude hold with Kalman filtering  
✅ **Complementary Filter** - Fuses gyro and accelerometer (99%/1%)  
✅ **Anti-Windup** - PID integral protection prevents overshooting  

### Safety Features
✅ **Failsafe** - Auto-disarm after 3 seconds signal loss  
✅ **Angle Limits** - Configurable max tilt angle  
✅ **Kill Switch** - Immediate disarm (switch 1)  
✅ **Arming Sequence** - Prevents accidental motor start  
✅ **Low Battery Warning** - Voltage monitoring (if connected)  

### User Interface
✅ **LED Status** - Visual feedback  
✅ **Buzzer Alerts** - Audio status/warnings  
✅ **Serial Debug** - Real-time telemetry output  
✅ **EEPROM Calibration** - Saves gyro calibration  

## Configuration

### PID Tuning (in `FlightController.ino`)
```cpp
// Attitude PID (adjust these for your quad)
const float kp = 2.0;          // Proportional gain
const float ki = 0.0001;       // Integral gain  
const float kd = 0.5;          // Derivative gain
const float kpZ = 2.0;         // Yaw P gain

// Altitude PID
float pid_p_gain_altitude = 14.0;
float pid_i_gain_altitude = 2.0;
float pid_d_gain_altitude = 7.5;
```

### Flight Parameters
```cpp
float hz = 140.0;              // Loop frequency (Hz)
int maxThrust = 1700;          // Max throttle (PWM)
int maxAngle = 180;            // Max tilt angle (degrees)
bool killAngle = true;         // Enable angle protection
```

### Radio Settings (MUST MATCH on both boards!)
```cpp
const uint64_t pipe = 0xF0F0F0F0E1LL;
radio.setChannel(35);          // Channel 35 = 2.435 GHz
```

## Documentation

📖 **[PROJECT_SUMMARY.md](PROJECT_SUMMARY.md)** - Complete documentation including:
- Detailed pin connections
- Pre-flight checklist
- PID tuning guide
- Troubleshooting
- Advanced modifications

📝 **[FIXES_AND_OPTIMIZATIONS.md](FIXES_AND_OPTIMIZATIONS.md)** - Complete changelog:
- All 17 bugs fixed
- Optimization details
- Code quality improvements
- Testing recommendations

## Control Mapping

### Flight Controller
- **Roll** - Right stick X-axis
- **Pitch** - Right stick Y-axis  
- **Yaw** - Left stick X-axis
- **Throttle** - Left stick Y-axis
- **Arm/Disarm** - Button 2 (hold 2 sec)
- **Calibrate** - Button 1 (hold 2 sec)
- **Kill Switch** - Switch 1 (immediate disarm)
- **Altitude Hold** - Switch 2 (throttle at mid-range)

## Troubleshooting

### Motors don't spin when armed
- Check ESC connections and calibration
- Verify PWM range matches ESCs (1000-2000μs)
- Test with serial monitor for errors

### Quad flips on takeoff
- **Wrong motor direction** - Check props/rotation
- **Incorrect motor mapping** - Verify pin connections
- **Gyro orientation wrong** - Remount MPU6050

### Radio disconnects frequently
- Check nRF24L01 antenna
- Add 10μF capacitor to nRF24L01 power
- Verify channel and pipe match on both boards
- Reduce distance for testing

### Altitude hold unstable
- Calibrate barometer at ground level
- Check MS5611 I2C wiring
- Reduce altitude PID gains
- Verify Kalman filter is working (debug output)

### Drifts during flight
- **Recalibrate gyro** on perfectly level surface
- Check for **vibration** - add damping
- Tune **integral gain** to eliminate drift

## Serial Debug Output

Enable detailed debugging:
```cpp
debugging(true);  // Add to setup()
```

Serial output (57600 baud) shows:
- Barometric pressure (raw & filtered)
- Kalman height estimate
- Vertical velocity
- PID errors
- Motor outputs

## Version History

**v2.0 - Fixed & Optimized** (Current)
- ✅ Fixed critical Kalman filter bug
- ✅ Added PID anti-windup protection
- ✅ Improved safety checks throughout
- ✅ Optimized floating-point precision
- ✅ Enhanced complementary filter
- ✅ Added comprehensive documentation

**v1.0 - Original** (DO NOT USE - has critical bugs)

## Contributing

Found an issue or have improvements?
1. Test thoroughly before submitting
2. Document all changes
3. Follow existing code style
4. Update documentation

## License

LGPL (GNU Lesser General Public License) - Free to modify and use.

## Disclaimer

⚠️ **This is experimental software for educational purposes.**

- Fly **at your own risk**
- No warranty or guarantee of safety
- Always follow local UAV regulations
- Test incrementally and safely
- Keep spare parts on hand
- Have fun, but be responsible!

---

## Next Steps

1. Read **PROJECT_SUMMARY.md** for complete setup guide
2. Review **FIXES_AND_OPTIMIZATIONS.md** to understand changes
3. Follow pre-flight checklist carefully
4. Start with props off, test all functions
5. First flight: hover at low altitude
6. Tune PID gains as needed
7. Enjoy flying! 🚁

**Questions?** Check the documentation or review the extensively commented source code.

**Happy Flying!** ✈️
