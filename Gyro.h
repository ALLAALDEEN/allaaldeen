#ifndef Gyro_h
#define Gyro_h

#include "Arduino.h"

struct Vec3
{
  float x, y, z;
  
  Vec3() : x(0), y(0), z(0) {}
  Vec3(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}
  
  Vec3 operator+(const Vec3& other) const {
    return Vec3(x + other.x, y + other.y, z + other.z);
  }
  
  Vec3 operator-(const Vec3& other) const {
    return Vec3(x - other.x, y - other.y, z - other.z);
  }
  
  Vec3 operator*(float scalar) const {
    return Vec3(x * scalar, y * scalar, z * scalar);
  }
  
  Vec3& operator=(const Vec3& other) {
    if (this != &other) {
      x = other.x;
      y = other.y;
      z = other.z;
    }
    return *this;
  }
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
  float Acc_totalVec;     // total vector of raw acceleration vector

  // Time per iteration
  double Time = 0;
  double prevTime = 0;
  bool countTime = false;

  // Scale factors
  const int ScaleAcc = 8192;                  // Datasheet value
  const int ScaleGyro = 65.5;                 // Datasheet value
  const float rad_to_deg = 180.0 / PI;        // Rad to Degree
  const float deg_to_rad = PI / 180.0;        // Degree to Rad
  double micro_to_sec = 0.000001;             // scaling factor micro to seconds
  float limZ = ScaleGyro / 100;               // Low Pass Filter for Gyro Z component

  bool GyroSet = true;                        // variable to check if absolute(accangle) has been set on first iteration

public:
  Vec3 error;

  Gyro() {
    GyroScaled = Vec3();
    Gyro_angle = Vec3();
    RawAcc = Vec3();
    RawGyro = Vec3();
    Acc_angle = Vec3();
    target = Vec3();
    cal = Vec3();
    GyroCal = Vec3();
    error = Vec3();
    Acc_totalVec = 0;
  }

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
  void setupwire();
};

#endif