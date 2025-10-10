# Flight Controller Code Fixes and Optimizations

## Critical Fixes Applied

### 1. **Kalman Filter Initialization Bug (CRITICAL)**
**Problem:** `initKalmanPosVel()` was called every loop iteration, resetting the covariance matrix and breaking the filter.

**Fix:** 
- Added `kalman_initialized` flag
- Call `initKalmanPosVel()` only once in `setup()`
- Only call `KalmanPosVel()` for updates in the main loop

**Impact:** This was causing the altitude estimation to never converge and lose all historical data.

### 2. **Gyro.cpp Function Naming Inconsistency**
**Problem:** Function declared as `setupWire()` but called as `SetupWire()` - case mismatch.

**Fix:** Standardized all function names to use consistent casing.

### 3. **Division by Zero Protection**
**Problem:** No safety checks in Kalman filter and battery voltage calculations.

**Fix:** 
- Added check: `if (abs(ic) < 0.0001) ic = 0.0001;` in Kalman filter
- Added check in battery voltage: `if ((R1 + R2) > 0.0)`
- Added checks in accelerometer angle calculation

### 4. **PID Integral Windup**
**Problem:** Integral terms could grow unbounded when errors persist.

**Fix:** Added integral clamping in `calculatePID()`:
```cpp
const float MAX_INTEGRAL = 200.0;
PID[1].x = constrain(PID[1].x, -MAX_INTEGRAL, MAX_INTEGRAL);
PID[1].y = constrain(PID[1].y, -MAX_INTEGRAL, MAX_INTEGRAL);
PID[1].z = constrain(PID[1].z, -MAX_INTEGRAL, MAX_INTEGRAL);
```

### 5. **Altitude Hold Logic Simplification**
**Problem:** Redundant condition checks and missing reset logic.

**Fix:**
- Simplified condition: `bool altitude_hold_active = (switch2 == 0 && thrust > 1400 && thrust < 1450);`
- Reset `pid_i_mem_altitude` when exiting altitude hold mode
- Reset `pid_output_altitude` when not in altitude hold

### 6. **Accelerometer Angle Calculation**
**Problem:** Could cause errors with extreme accelerations.

**Fix:** Added safety checks before `atan()` calls and improved complementary filter:
```cpp
if (RawAcc.z > -100 && Acc_totalVec > 0.1 && Acc_totalVec < 2.0)
```

## Precision Optimizations

### 7. **Floating Point Consistency**
**Problem:** Mixed use of integer and float constants causing precision loss.

**Fix:** 
- Changed all constants to explicit float: `1.0`, `0.0`, `140.0`
- Updated timeslice: `#define TIMESLICE 0.00714285` (1/140)

### 8. **Thrust Range Validation**
**Problem:** No bounds checking on calculated thrust values.

**Fix:** Added constraints:
```cpp
thrust_2 = constrain(thrust_2, MINarmed, maxThrust);
package.thrust = constrain(package.thrust, 1000, 2000);
```

### 9. **Gyro Calibration Stability**
**Problem:** No delays during calibration sampling.

**Fix:** Added `delay(1)` in calibration loop and `delay(5)` in angle calibration for stable readings.

### 10. **Controller Smoothing**
**Problem:** Exponential filter factor was set to 1 (no smoothing).

**Fix:** Changed to factor of 10 for proper smoothing:
```cpp
degerexpo.begin(SMOOTHED_EXPONENTIAL, 10);
```

### 11. **Analog Range Correction**
**Problem:** Used 1013 instead of 1023 for analog inversion.

**Fix:** `smoothyr = 1023 - analogRead(YR_pin);`

## Code Quality Improvements

### 12. **Debug Output Enhancement**
**Problem:** Debug prints always active, cluttering serial output.

**Fix:** Added conditional printing:
```cpp
void Print() {
  if (dBugging) {
    // print statements
  }
}
```

### 13. **Variable Name Corrections**
- Fixed typo: `altutude_hold_pressure` → `altitude_hold_pressure`
- Fixed typo: `pressure_total_avarage` → `pressure_total_average`

### 14. **Complementary Filter Bounds**
**Problem:** Accelerometer data used even during high acceleration (crashes, impacts).

**Fix:** Added acceleration magnitude check (0.1g to 2.0g range) to only trust accelerometer during stable flight.

### 15. **Arming/Disarming Logic**
**Problem:** Could only arm, not disarm with button.

**Fix:** 
```cpp
if (armed == false) armed = true;
else armed = false;  // Allow disarming too
```

## Performance Optimizations

### 16. **Reduced Barometer Read Frequency**
Already implemented correctly - reads every 10 loops, reducing I2C overhead.

### 17. **Motor Mixing Calculation**
Simplified to always use the same mixing equations, with conditional thrust source selection.

## Remaining Recommendations

### For Future Optimization:
1. **Add Failsafe Modes**: Implement GPS return-to-home or auto-land on signal loss
2. **Battery Monitoring**: Use `calculate_battery()` function (currently unused)
3. **Motor Test Mode**: Add pre-flight motor test sequence
4. **Data Logging**: Log flight data to SD card for analysis
5. **Rate Limiting**: Add rate limits to prevent too-aggressive maneuvers
6. **LED Status Indicators**: Use LED for armed/disarmed/error states
7. **EEPROM PID Storage**: Save PID values to EEPROM for tuning persistence

## Testing Checklist

Before flight testing:
- [ ] Verify motor directions (props off!)
- [ ] Test radio range and failsafe
- [ ] Calibrate gyro on level surface
- [ ] Test arming/disarming sequence
- [ ] Verify angle limits work
- [ ] Test altitude hold at low altitude
- [ ] Check battery monitoring
- [ ] Verify PID gains are appropriate for your frame
- [ ] Test with props off, motors should respond to stick inputs
- [ ] Start with conservative PID values and tune gradually

## Safety Notes

⚠️ **WARNING**: This is a flight controller for an aircraft. Improper configuration can cause injury or property damage.

1. Always test with props removed first
2. Have a safety observer during test flights
3. Test in open areas away from people and property
4. Start with low PID gains and increase gradually
5. Monitor battery voltage during flight
6. Implement a geofence for testing
7. Keep spare props and parts on hand

## File Structure

```
FlightController/
├── FlightController.ino  (Main flight controller code)
├── Gyro.cpp              (MPU6050 gyroscope/accelerometer driver)
├── Gyro.h                (Gyro class header)
└── libraries/
    └── Smoothed/         (Smoothing filter library)

Controller/
└── Controller.ino        (RC transmitter code)
```

## Dependencies

Required Arduino Libraries:
- Servo
- SPI
- Wire
- EEPROM
- nRF24L01 (RF24)
- MS5611 (barometer)
- Smoothed (included)

## Compilation Notes

- Target: Arduino Nano/Uno or compatible (ATmega328P)
- Upload speed: 57600 baud
- Ensure all libraries are installed via Library Manager
