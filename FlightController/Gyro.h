#ifndef Gyro_h
#define Gyro_h

#include "Arduino.h"

struct Vec3
{
  float x, y, z;
};

class Gyro
{
  private:
    Vec3 GyroScaled;      // angular speed in deg/sec
    Vec3 Gyro_angle;      // gyro integral in deg
    Vec3 RawAcc;          // raw values acceleration vector
    Vec3 RawGyro;         // raw values angular velocity
    float tmp = 0;        // temperature (unused)

    Vec3 Acc_angle;       // acceleration angle in deg
    Vec3 target;          // target angle in deg
    Vec3 cal;             // calibration angle in deg
    Vec3 GyroCal;         // calibration for raw gyro values
    float Acc_totalVec;   // total vector of raw-acceleration vector

    // Time per Iteration
    double Time = 0;
    double prevTime = 0;
    bool countTime = false;

    // Scale factors
    const float ScaleAcc = 4096.0;             // For AFS_SEL=2 (±8g)
    const float ScaleGyro = 65.5;              // For FS_SEL=1 (±500°/s)
    const float rad_to_deg = 180.0 / 3.141592654;
    const float deg_to_rad = 3.141592654 / 180.0;
    const double micro_to_sec = 0.000001;
    float limZ = ScaleGyro / 100.0;            // Low Pass Filter for Gyro Z component

    bool GyroSet = true;                       // Check if absolute acc angle has been set

    void setupWire();                          // FIX: Consistent naming

  public:
    Gyro() {}

    Vec3 error;

    // Function prototypes
    void zeroYaw(bool lt);
    void calculateError();
    void setTarget(Vec3 Target);
    void setCalibration(Vec3 Cal);
    void readingMPU();
    void calibrateGyro();
    Vec3 calibrate(int n);
    void calculateAngle();
    void SetupWire(double TIME);   // TIME = time per iteration = 1/hz
    void SetupWire();
};

#endif
