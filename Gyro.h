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
  Vec3 GyroScaled;        // angular speed in deg/sec
  Vec3 Gyro_angle;        // gyro integral in deg

  Vec3 RawAcc;            // raw values acceleration vector
  Vec3 RawGyro;           // raw values angular velocity
  float tmp = 0;          // temperature (unused)

  Vec3 Acc_angle;         // acceleration angle in deg
  Vec3 target;            // target angle in deg
  Vec3 cal;               // calibration angle in deg
  Vec3 GyroCal;           // calibration for raw gyro values 
  float Acc_totalVec;     // total vector of raw-acceleration vector

  // Time per Iteration
  double Time = 0;
  double prevTime = 0;
  bool countTime = false;

  // Scale factors - improved precision
  const float ScaleAcc = 4096.0;                    // Updated for ±8g range
  const float ScaleGyro = 65.5;                     // ±500°/s range
  const float rad_to_deg = 180.0 / 3.141592654;     // Rad to Degree
  const float deg_to_rad = 3.141592654 / 180.0;     // Degree to Rad
  const double micro_to_sec = 0.000001;             // scaling factor micro to seconds
  float limZ = ScaleGyro / 100.0;                   // Low Pass Filter for Gyro Z component

  bool GyroSet = true;    // variable to check if absolute(accangle) has been set on first iteration

public:
  Gyro()
  {
    // Initialize all vectors to zero
    GyroScaled = {0, 0, 0};
    Gyro_angle = {0, 0, 0};
    RawAcc = {0, 0, 0};
    RawGyro = {0, 0, 0};
    Acc_angle = {0, 0, 0};
    target = {0, 0, 0};
    cal = {0, 0, 0};
    GyroCal = {0, 0, 0};
    Acc_totalVec = 0;
  }

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
  void SetupWire(double TIME);    // TIME = 'time per iteration' = 1/hz
  void SetupWire();

private:
  void setupwire();
};

#endif