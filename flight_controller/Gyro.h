#ifndef Gyro_h
#define Gyro_h

#include <Arduino.h>

struct Vec3 {
  float x, y, z;
};

class Gyro {
private:
  Vec3 GyroScaled;     // angular speed in deg/sec
  Vec3 Gyro_angle;     // gyro integral in deg

  Vec3 RawAcc;         // raw acceleration vector
  Vec3 RawGyro;        // raw angular velocity
  float tmp = 0;       // temperature (unused)

  Vec3 Acc_angle;      // acceleration angle in deg
  Vec3 target;         // target angle in deg
  Vec3 cal;            // calibration angle in deg
  Vec3 GyroCal;        // calibration for raw gyro values
  float Acc_totalVec;  // magnitude of acceleration vector

  // Time per iteration
  double Time = 0;
  unsigned long prevTime = 0;
  bool countTime = false;

  // Scale
  const int    ScaleAcc = 8192;                 // Datasheet
  const int    ScaleGyro = 65.5;                // Datasheet
  const float  rad_to_deg = 180.0 / 3.141592654;
  const float  deg_to_rad = 3.141592654 / 180.0;
  double       micro_to_sec = 0.000001;         // scaling factor micro to seconds
  float        limZ = ScaleGyro / 100.0;        // Low Pass Filter for Gyro Z component

  bool GyroSet = true;                          // flag to set absolute acc angle on first iteration

public:
  Gyro() {}

  Vec3 error;

  // API
  void zeroYaw(bool lt);
  void calculateError();
  void setTarget(Vec3 Target);
  void setCalibration(Vec3 Cal);
  void readingMPU();
  void calibrateGyro();
  Vec3 calibrate(int n);
  void calculateAngle();
  void SetupWire(double TIME);  // TIME = 1/hz
  void SetupWire();

private:
  void setupwire();
};

#endif