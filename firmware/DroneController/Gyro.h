/**
 * Gyro.h
 *
 * Minimal MPU6050 wrapper used by the flight controller.
 */

#pragma once

#include <Arduino.h>

struct Vec3 {
  float x;
  float y;
  float z;
};

class Gyro {
public:
  Gyro() = default;

  Vec3 error;

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
  void setupwire();

private:
  Vec3 GyroScaled{0, 0, 0};
  Vec3 Gyro_angle{0, 0, 0};

  Vec3 RawAcc{0, 0, 0};
  Vec3 RawGyro{0, 0, 0};

  float tmp = 0.0f;

  Vec3 Acc_angle{0, 0, 0};
  Vec3 target{0, 0, 0};
  Vec3 cal{0, 0, 0};
  Vec3 GyroCal{0, 0, 0};
  float Acc_totalVec = 0.0f;

  double Time = 0.0;
  double prevTime = 0.0;
  bool countTime = false;

  const int ScaleAcc = 8192;
  const float ScaleGyro = 65.5f;
  const float rad_to_deg = 180.0f / 3.141592654f;
  const float deg_to_rad = 3.141592654f / 180.0f;
  double micro_to_sec = 0.000001;
  float limZ = ScaleGyro / 100.0f;

  bool GyroSet = true;
};
