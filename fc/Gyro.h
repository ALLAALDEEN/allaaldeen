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
  float tmp = 0;       // raw temperature (unused)

  Vec3 Acc_angle;      // acceleration angle in deg
  Vec3 target;         // target angle in deg
  Vec3 cal;            // calibration angle in deg
  Vec3 GyroCal;        // calibration for raw gyro values
  float Acc_totalVec;  // total vector of raw acceleration

  // Time per iteration
  double Time = 0;
  double prevTime = 0;
  bool countTime = false;

  // Scale
  static constexpr int    ScaleAcc = 8192;       // datasheet
  static constexpr float  ScaleGyro = 65.5f;     // datasheet
  static constexpr float  rad_to_deg = 180.0f / 3.141592654f;
  static constexpr float  deg_to_rad = 3.141592654f / 180.0f;
  const double micro_to_sec = 1e-6;              // micros to seconds
  float limZ = ScaleGyro / 100.0f;               // LPF threshold for gyro Z

  bool GyroSet = true;                           // set absolute acc-angle on first iteration

  void setupwire();
  void readingMPU();
  void calculateAngle();
  void calibrateGyro();

public:
  Gyro() {}

  Vec3 error;

  void zeroYaw(bool lt);
  void calculateError();
  void setTarget(Vec3 Target);
  void setCalibration(Vec3 Cal);
  Vec3 calibrate(int n);
  void SetupWire(double TIME);   // TIME = 1/hz
  void SetupWire();
};

#endif
