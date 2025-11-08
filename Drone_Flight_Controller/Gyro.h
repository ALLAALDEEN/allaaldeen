#ifndef Gyro_h
#define Gyro_h

#include "Arduino.h"

struct Vec3
{
  float x;
  float y;
  float z;
};

class Gyro
{
private:
  Vec3 GyroScaled;
  Vec3 Gyro_angle;

  Vec3 RawAcc;
  Vec3 RawGyro;
  float tmp = 0;

  Vec3 Acc_angle;
  Vec3 target;
  Vec3 cal;
  Vec3 GyroCal;
  float Acc_totalVec;

  double Time = 0;
  double prevTime = 0;
  bool countTime = false;

  const int ScaleAcc = 8192;
  const int ScaleGyro = 65.5;
  const float rad_to_deg = 180.0f / 3.141592654f;
  const float deg_to_rad = 3.141592654f / 180.0f;
  double micro_to_sec = 0.000001;
  float limZ = ScaleGyro / 100.0f;

  bool GyroSet = true;

  void setupwire();

public:
  Gyro() = default;

  Vec3 error{0, 0, 0};

  void zeroYaw(bool lt);
  void calculateError();
  void setTarget(Vec3 Target);
  void setCalibration(Vec3 Cal);
  void readingMPU();
  void calibrateGyro();
  Vec3 calibrate(int n);
  void calculateAngle();
  void SetupWire(double TIME);
  void SetupWire();
};

#endif
