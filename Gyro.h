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
  float tmp = 0;        // useless

  Vec3 Acc_angle;       // acceleration angle in deg
  Vec3 target;          // target angle in deg/sec
  Vec3 cal;             // calibration angle in deg/sec
  Vec3 GyroCal;         // calibration for rawgyro values
  float Acc_totalVec;   // total vector of raw-acceleration vector

  // Time per Iteration
  double Time = 0;
  double prevTime = 0;
  bool countTime = false;

  // Scale
  const int ScaleAcc = 8192;                 // Datasheet
  const int ScaleGyro = 65.5;                // Datasheet
  const float rad_to_deg = 180 / 3.141592654;// Rad to Degree, because atan() returns values in rad
  const float deg_to_rad = 3.141592654 / 180;// Degree to Rad, because atan() only accepts rad as an argument
  double micro_to_sec = 0.000001;            // scaling factor micro to seconds
  float limZ = ScaleGyro / 100;              // Low Pass Filter for Gyro Z component

  bool GyroSet = true;                       // variable to check if absolute(accangle) has been set on first iteration

public:
  Gyro() {}

  Vec3 error;

  // function prototypes
  void zeroYaw(bool lt);
  void calculateError();
  void setTarget(Vec3 Target);
  void setCalibration(Vec3 Cal);
  void readingMPU();
  void calibrateGyro();
  Vec3 calibrate(int n);
  void calculateAngle();
  void SetupWire(double TIME);      // TIME = 'time per iteration' = 1/hz
  void SetupWire();
  void setupwire();
};

#endif
