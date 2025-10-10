#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>
#include <Wire.h>
#include "Gyro.h"
#include "MS5611.h"
#include "Smoothed.h"

// Barometer
MS5611 MS5611(0x77);

// Radio (CE, CSN)
RF24 radio(4, 10);
const uint64_t pipe = 0xF0F0F0F0E1LL;

// IO
const int BUZZER = 8;
const int LED = 7;

const int flPIN = 3;
const int frPIN = 5;
const int rrPIN = 6;
const int rlPIN = 9;

// Smoothing
Smoothed<float> smooth;

// Flight state
bool but1 = true, but2 = true, switch1 = true, switch2 = true;
byte baroCounter = 0;

struct Package {
  int   thrust = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  int   id = 0;
  bool  but1 = 1;
  bool  but2 = 1;
  bool  switch1 = 1;
  bool  switch2 = 1;
};

Package package;
Gyro gyro;

Servo ESCfl, ESCfr, ESCrl, ESCrr;

// PID parameters
const float kp = 2.0f;
const float ki = 0.0001f;
const float kd = 0.5f;
const float kpZ = 2.0f;

// Altitude PID parameters
float pid_p_gain_altitude = 14.0f;
float pid_i_gain_altitude = 2.0f;
float pid_d_gain_altitude = 7.5f;
int   pid_max_altitude    = 400;

float pid_error_gain_altitude = 0, pid_throttle_gain_altitude = 0;
float ground_pressure = 0, altitude_hold_pressure = 0;
float pid_i_mem_altitude = 0, pid_altitude_setpoint = 0, pid_altitude_input = 0, pid_output_altitude = 0, pid_last_altitude_d_error = 0;
uint8_t parachute_rotating_mem_location = 0;
int32_t parachute_buffer[35] = {0}, parachute_throttle = 0;
float pressure_parachute_previous = 0;
int32_t pressure_rotating_mem[50] = {0}, pressure_total_avarage = 0;
uint8_t pressure_rotating_mem_location = 0;
float pressure_rotating_mem_actual = 0;
float actual_pressure = 0, pid_error_temp = 0;
uint8_t manual_altitude_change = 0;
int16_t manual_throttle = 0;
byte hold = 0;
float actual_pressure_2 = 0;

// Sensitivity
float sensiX = -0.45f;
float sensiY = 0.45f;
float sensiZ = -0.01f;
float sensiThrust = 1.1f;

// Filters
int lowPassX = 5;
int lowPassY = 5;
int lowPassZ = 10;

// Frequency
float hz = 140.0f;

// Motor limits
int pMAX = 2000;
int pMIN = 1000;
int MINarmed = 1050;
int maxThrust = 1700;

int maxAngle = 180;
bool killAngle = true;

int MAX = pMAX;
int MIN = pMIN;
int thrust = pMIN;
int thrust_2 = thrust;
int killSwitch = 0;

float calCount = 0;
float NoDataCount = 0;
float armingCounter = 0;

bool dBugging = false;
bool armed = false;

double timepi = 0;      // seconds per loop
unsigned long prevTime = 0;

const float sec_to_micro = 1000000.0f;
const float micro_to_sec = 1e-6f;
const float micro_to_ms = 0.001f;
const int   sec_to_ms = 1000;

int FrontRight = thrust;
int FrontLeft  = thrust;
int RearRight  = thrust;
int RearLeft   = thrust;

Vec3 PID[3] = { {0,0,0}, {0,0,0}, {0,0,0} };
Vec3 target = {0,0,0};
Vec3 cal = {0,0,0};
Vec3 rawCal = {0,0,0};
Vec3 prevError = {0,0,0};

// Kalman filter structs
struct quad_properties {
  float height;
  float kalmanvel_z;
  float baro_height;
};
struct quad_properties quadprops;

struct matrix2x2 {
  float m11; float m21;
  float m12; float m22;
};
struct matrix2x2 current_prob;

// Function prototypes
void Print();
void readEEPROM();
bool receiveRadio();
void checkStatus();
void calculatePID();
void calculateVelocities();
void waitLoop();
void runMotors();
void stopMotors();
void resetYaw();
void calculate_pressure();
void calculate_battery();
void ledBlink(int t);
void KalmanPosVel();
void initKalmanPosVel();

void setup() {
  Serial.begin(57600);
  prevTime = micros();
  timepi = 1.0 / hz;

  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);

  tone(BUZZER, 1000 , 300); ledBlink(300); delay(100);
  tone(BUZZER, 1600 , 700); ledBlink(700); delay(100);
  tone(BUZZER, 2000 , 200); ledBlink(200);

  ESCfl.attach(flPIN, 1000, 2000);
  ESCfr.attach(frPIN, 1000, 2000);
  ESCrl.attach(rlPIN, 1000, 2000);
  ESCrr.attach(rrPIN, 1000, 2000);
  stopMotors();
  delay(500);
  Serial.println("Motors attached");

  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(90);                // Channel 90 (2.490 GHz)
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  Serial.println("Radio OK");

  readEEPROM();
  gyro.SetupWire(timepi);
  delay(500);
  tone(BUZZER, 2000 , 200); ledBlink(200);

  MS5611.begin();
  MS5611.setOversampling(OSR_LOW);
  smooth.begin(SMOOTHED_AVERAGE, 10);

  initKalmanPosVel();
}

void loop() {
  receiveRadio();
  checkStatus();
  gyro.setTarget(target);
  gyro.setCalibration(cal);
  calculate_pressure();
  gyro.calculateError();
  calculatePID();
  calculateVelocities();
  runMotors();
  Print();
  waitLoop();
}

// ---------------- PID Calculation ----------------
void calculatePID() {
  if (!armed) {
    resetYaw();
  }

  if (armed) {
    PID[0].x = gyro.error.x * kp;
    PID[0].y = gyro.error.y * kp;
    PID[0].z = gyro.error.z * kpZ;

    PID[1].x += gyro.error.x * timepi * ki;
    PID[1].y += gyro.error.y * timepi * ki;
    PID[1].z += gyro.error.z * timepi * ki;

    PID[2].x = kd * (gyro.error.x - prevError.x) / timepi;
    PID[2].y = kd * (gyro.error.y - prevError.y) / timepi;
    PID[2].z = kd * (gyro.error.z - prevError.z) / timepi;

    prevError = gyro.error;
  } else {
    PID[0] = {0,0,0};
    PID[1] = {0,0,0};
    PID[2] = {0,0,0};
    prevError = {0,0,0};
  }
}

// ---------------- Motor Velocity Mixing ----------------
void calculateVelocities() {
  thrust_2 = (1450 + pid_output_altitude + manual_throttle);
  if (switch2 == 0 && thrust < 1450 && thrust > 1400) {
    RearLeft   = thrust_2 - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
    RearRight  = thrust_2 + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
    FrontLeft  = thrust_2 - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
    FrontRight = thrust_2 + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
  } else {
    RearLeft   = thrust - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
    RearRight  = thrust + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
    FrontLeft  = thrust - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
    FrontRight = thrust + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
  }
}

// ---------------- Motor Output ----------------
void runMotors() {
  MIN = armed ? MINarmed : pMIN;

  RearLeft  = constrain(RearLeft,  MIN, MAX);
  RearRight = constrain(RearRight, MIN, MAX);
  FrontLeft = constrain(FrontLeft, MIN, MAX);
  FrontRight= constrain(FrontRight,MIN, MAX);

  if (armed) {
    ESCfl.writeMicroseconds(FrontLeft);
    ESCfr.writeMicroseconds(FrontRight);
    ESCrl.writeMicroseconds(RearLeft);
    ESCrr.writeMicroseconds(RearRight);
  } else {
    stopMotors();
  }
}

void stopMotors() {
  ESCfl.writeMicroseconds(1000);
  ESCfr.writeMicroseconds(1000);
  ESCrl.writeMicroseconds(1000);
  ESCrr.writeMicroseconds(1000);

  MIN = pMIN;
  FrontRight = pMIN;
  FrontLeft  = pMIN;
  RearLeft   = pMIN;
  RearRight  = pMIN;
}

// ---------------- RF Data Reception ----------------
bool receiveRadio() {
  if (radio.available()) {
    radio.read(&package, sizeof(package));
    but1 = package.but1;
    but2 = package.but2;
    switch1 = package.switch1;
    switch2 = package.switch2;

    if (package.thrust != 0) {
      if (fabs(package.z) < lowPassZ) package.z = 0;
      if (fabs(package.x) < lowPassX) package.x = 0;
      if (fabs(package.y) < lowPassY) package.y = 0;

      target.x = package.x * sensiX;
      target.y = package.y * sensiY;
      if (armed) target.z += package.z * sensiZ;

      thrust = (int)(package.thrust * sensiThrust);
      thrust = constrain(thrust, MIN, maxThrust);

      NoDataCount = 0;
      return true;
    } else {
      NoDataCount += timepi;
      return false;
    }
  } else {
    NoDataCount += timepi;
    return false;
  }
}

// ---------------- Status and Safety ----------------
void checkStatus() {
  if (switch1 == 0) { // emergency stop
    stopMotors();
    armed = false;
  }

  if (gyro.error.z > 180 || gyro.error.z < -180) resetYaw();

  if (NoDataCount > 3) killSwitch = 2;

  if (abs(gyro.error.x) > maxAngle || abs(gyro.error.y) > maxAngle) {
    if (killAngle) killSwitch = 1;
  }

  if (killSwitch > 0) {
    stopMotors();
    while (killSwitch > 0) {
      tone(BUZZER, 1000 , 300); ledBlink(300); delay(2000);
      if (killSwitch == 2 && radio.available()) {
        delay(500);
        if (radio.available()) {
          tone(BUZZER, 1500, 1000); ledBlink(1000);
          killSwitch = 0;
          armed = false;
        }
      }
    }
  }

  if (but2 == 0) {
    armingCounter += timepi;
    resetYaw();
    if (armingCounter > 2) {
      tone(BUZZER, 1500, 500); ledBlink(500);
      if (!armed) armed = true;
      armingCounter = 0;
    }
  } else {
    armingCounter = 0;
  }

  if (but1 == 0) {
    calCount += timepi;
    if (calCount > 2) {
      stopMotors();
      tone(BUZZER, 1200, 100); ledBlink(100); delay(300);
      tone(BUZZER, 1200, 200); ledBlink(200);

      cal = gyro.calibrate(1000);
      EEPROM.put(10, (float)cal.x);
      EEPROM.put(15, (float)cal.y);
      delay(500);
      gyro.setCalibration(cal);
      tone(BUZZER, 2200, 200); ledBlink(200); delay(1000);
      calCount = 0;
    }
  } else {
    calCount = 0;
  }
}

void calculate_battery() {
  // Example voltage divider on A0 -> compute Vin if needed
  // int real_voltage = analogRead(A0);
  // float vout = (real_voltage * 5.0f) / 1023.0f;
  // float R1 = 1500.0f, R2 = 1000.0f;
  // float vin = vout / (R2 / (R1 + R2));
}

void waitLoop() {
  while ((micros() - prevTime) < (unsigned long)(timepi * sec_to_micro)) {}
  prevTime = micros();
}

void ledBlink(int t) {
  digitalWrite(LED, HIGH);
  delay(t);
  digitalWrite(LED, LOW);
}

void readEEPROM() {
  EEPROM.get(10, cal.x);
  EEPROM.get(15, cal.y);
}

void resetYaw() {
  gyro.zeroYaw(true);
  target.z = 0;
}

void Print() {
  Serial.print("actual_pressure= ");
  Serial.print(actual_pressure);
  Serial.print("\tkalman_vel_z= ");
  Serial.print(actual_pressure_2);
  Serial.println();
}

// ---------------- Barometer / Altitude hold ----------------
void calculate_pressure() {
  const byte baroReadDivider = 10; // read pressure every N loops
  if (baroCounter == 0) {
    MS5611.read();
    smooth.add(MS5611.getPressure());
    baroCounter = baroReadDivider;
  }
  baroCounter--;

  actual_pressure = smooth.get();
  quadprops.baro_height = actual_pressure;
  KalmanPosVel();
  actual_pressure_2 = quadprops.kalmanvel_z;

  if (switch2 == 0 && thrust > 1400 && thrust < 1450) {
    if (manual_altitude_change == 1) {
      pressure_parachute_previous = actual_pressure * 10.0f;
    }

    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10.0f - pressure_parachute_previous;
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];
    pressure_parachute_previous = actual_pressure * 10.0f;
    parachute_rotating_mem_location++;
    if (parachute_rotating_mem_location == 30) parachute_rotating_mem_location = 0;

    if (switch2 == 0 && thrust < 1450 && thrust > 1400) {
      if (hold == 0) { pid_altitude_setpoint = actual_pressure; hold = 1; }

      manual_altitude_change = 0;
      manual_throttle = 0;
      if (thrust > 1450) {
        manual_altitude_change = 1;
        pid_altitude_setpoint = actual_pressure;
        manual_throttle = (thrust - 1450) / 3;
      }
      if (thrust < 1400) {
        manual_altitude_change = 1;
        pid_altitude_setpoint = actual_pressure;
        manual_throttle = (thrust - 1400) / 5;
      }

      pid_altitude_input = actual_pressure;
      pid_error_temp = pid_altitude_input - pid_altitude_setpoint;

      pid_error_gain_altitude = 0;
      if (pid_error_temp > 10 || pid_error_temp < -10) {
        pid_error_gain_altitude = (fabs(pid_error_temp) - 10) / 20.0f;
        if (pid_error_gain_altitude > 3) pid_error_gain_altitude = 3;
      }

      pid_i_mem_altitude += (pid_i_gain_altitude / 100.0f) * pid_error_temp;
      if (pid_i_mem_altitude > pid_max_altitude) pid_i_mem_altitude = pid_max_altitude;
      else if (pid_i_mem_altitude < -pid_max_altitude) pid_i_mem_altitude = -pid_max_altitude;

      pid_output_altitude = (100.0f * (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle);
      if (pid_output_altitude > pid_max_altitude) pid_output_altitude = pid_max_altitude;
      else if (pid_output_altitude < -pid_max_altitude) pid_output_altitude = -pid_max_altitude;
    }
  } else {
    hold = 0;
  }
}
