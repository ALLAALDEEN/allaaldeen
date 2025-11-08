/**
 * Drone flight controller firmware.
 *
 * The sketch is split across multiple compilation units in this folder:
 *  - Drone_Flight_Controller.ino (main control loop + high level logic)
 *  - Barometer.ino (barometric altitude hold logic)
 *  - kalman_filter.ino (altitude Kalman filter implementation)
 *  - Gyro.cpp / Gyro.h (IMU driver and attitude estimation)
 *
 * The Arduino build process concatenates all *.ino files in lexicographical
 * order before compilation, so cross-file globals remain available.
 */

#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <EEPROM.h>
#include "Gyro.h"
#include <Smoothed.h>
#include <Wire.h>
#include "MS5611.h"

// MS5611 barometer (I2C address 0x77 by default)
MS5611 barometer(0x77);

// NRF24L01 radio definition
RF24 radio(4, 10);
constexpr uint64_t RADIO_PIPE = 0xF0F0F0F0E1LL;
constexpr uint8_t RADIO_CHANNEL = 115;  // Valid channel range: 0 - 125

// Controller state
bool but1, but2, switch1, switch2;
byte counter = 0;

struct Package
{
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

Servo ESCfl;
Servo ESCfr;
Servo ESCrl;
Servo ESCrr;

// Adjustable PID parameters
const float kp = 2.0f;
const float ki = 0.0001f;
const float kd = 0.5f;
const float kpZ = 2.0f;

float pid_p_gain_altitude = 14.0f;
float pid_i_gain_altitude = 2.0f;
float pid_d_gain_altitude = 7.5f;
int pid_max_altitude = 400;

float pid_error_gain_altitude;
float pid_throttle_gain_altitude;
float ground_pressure;
float altutude_hold_pressure;
float pid_i_mem_altitude;
float pid_altitude_setpoint;
float pid_altitude_input;
float pid_output_altitude;
float pid_last_altitude_d_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35];
int32_t parachute_throttle;
float pressure_parachute_previous;
int32_t pressure_rotating_mem[50];
int32_t pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;
float actual_pressure;
float pid_error_temp;
uint8_t manual_altitude_change;
int16_t manual_throttle;
byte hold;
float actual_pressure_2;

// Radio sensitivity configuration
float sensiX = -0.45f;
float sensiY = 0.45f;
float sensiZ = -0.01f;
float sensiThrust = 1.1f;

// Radio input low-pass thresholds
int lowPassX = 5;
int lowPassY = 5;
int lowPassZ = 10;

// Control loop frequency (~270 Hz max achievable, configured 140 Hz)
float hz = 140.0f;

// ESC command bounds
int pMAX = 2000;
int pMIN = 1000;
int MINarmed = 1050;
int maxThrust = 1700;

int maxAngle = 180;
bool killAngle = true;

const int flPIN = 3;
const int frPIN = 5;
const int rrPIN = 6;
const int rlPIN = 9;

Smoothed<float> smooth;

// Battery voltage measurement
float vout = 0.0f;
float vin = 0.0f;
int real_voltage = 0;
float R1 = 1500.0f;
float R2 = 1000.0f;

const int BUZZER = 8;
const int LED = 7;

int MAX = pMAX;
int MIN = pMIN;
int thrust = pMIN;
int thrust_2 = thrust;
int killSwitch = 0;

float calCount = 0.0f;
float NoDataCount = 0.0f;
float armingCounter = 0.0f;

bool dBugging = false;
bool armed = false;

// Timing helpers
double timepi = 0.0;
long prevTime = 0;

const float sec_to_micro = 1000000.0f;
const float micro_to_sec = 1.0f / 1000000.0f;
const float micro_to_ms = 0.001f;
const int sec_to_ms = 1000;

int FrontRight = thrust;
int FrontLeft = thrust;
int RearRight = thrust;
int RearLeft = thrust;

Vec3 PID[3] = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}
};

Vec3 target = {0, 0, 0};
Vec3 cal = {0, 0, 0};
Vec3 rawCal = {0, 0, 0};
Vec3 prevError = {0, 0, 0};

// Kalman filter structs (defined in kalman_filter.ino)
struct quad_properties {
  float height;
  float kalmanvel_z;
  float baro_height;
};

struct matrix2x2 {
  float m11;
  float m21;
  float m12;
  float m22;
};

quad_properties quadprops;
matrix2x2 current_prob;

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
void led(int t);
void KalmanPosVel();
void initKalmanPosVel();
void debugging(bool dBug);

void setup() {
  Serial.begin(57600);
  debugging(false);
  prevTime = micros();
  timepi = 1.0 / hz;

  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);

  tone(BUZZER, 1000, 300);
  led(300);
  delay(100);
  tone(BUZZER, 1600, 700);
  led(700);
  delay(100);
  tone(BUZZER, 2000, 200);
  led(200);

  ESCfl.attach(flPIN, 1000, 2000);
  ESCfr.attach(frPIN, 1000, 2000);
  ESCrl.attach(rlPIN, 1000, 2000);
  ESCrr.attach(rrPIN, 1000, 2000);
  stopMotors();

  delay(500);
  Serial.println(F("Motors\tattached"));

  if (!radio.begin()) {
    Serial.println(F("Radio init failed"));
  } else {
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.setChannel(RADIO_CHANNEL);
    radio.openReadingPipe(1, RADIO_PIPE);
    radio.startListening();
    Serial.println(F("Radio\tOK"));
  }

  readEEPROM();
  gyro.SetupWire(timepi);
  initKalmanPosVel();

  delay(500);
  tone(BUZZER, 2000, 200);
  led(200);

  if (!barometer.begin()) {
    Serial.println(F("MS5611 init failed"));
  } else {
    barometer.setOversampling(OSR_LOW);
  }

  smooth.begin(SMOOTHED_AVERAGE, 10);
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
    PID[0] = {0, 0, 0};
    PID[1] = {0, 0, 0};
    PID[2] = {0, 0, 0};
    prevError = {0, 0, 0};
  }
}

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

void runMotors() {
  MIN = armed ? MINarmed : pMIN;

  if (RearLeft < MIN) RearLeft = MIN;
  if (RearLeft > MAX) RearLeft = MAX;

  if (RearRight < MIN) RearRight = MIN;
  if (RearRight > MAX) RearRight = MAX;

  if (FrontLeft < MIN) FrontLeft = MIN;
  if (FrontLeft > MAX) FrontLeft = MAX;

  if (FrontRight < MIN) FrontRight = MIN;
  if (FrontRight > MAX) FrontRight = MAX;

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
  ESCfl.writeMicroseconds(0);
  ESCfr.writeMicroseconds(0);
  ESCrl.writeMicroseconds(0);
  ESCrr.writeMicroseconds(0);

  MIN = pMIN;

  FrontRight = pMIN;
  FrontLeft  = pMIN;
  RearLeft   = pMIN;
  RearRight  = pMIN;
}

bool receiveRadio() {
  if (radio.available()) {
    radio.read(&package, sizeof(package));
    but1 = package.but1;
    but2 = package.but2;
    switch1 = package.switch1;
    switch2 = package.switch2;

    if (package.thrust != 0) {
      if (package.z < lowPassZ && package.z > -lowPassZ) package.z = 0;
      if (package.x < lowPassX && package.x > -lowPassX) package.x = 0;
      if (package.y < lowPassY && package.y > -lowPassY) package.y = 0;

      target.x = package.x * sensiX;
      target.y = package.y * sensiY;

      if (armed) {
        target.z += package.z * sensiZ;
      }

      thrust = package.thrust * sensiThrust;

      if (thrust < MIN) thrust = MIN;
      if (thrust > maxThrust) thrust = maxThrust;

      NoDataCount = 0;
      return true;
    }

    NoDataCount += timepi;
    return false;
  }

  NoDataCount += timepi;
  return false;
}

void checkStatus() {
  if (switch1 == 0) {
    stopMotors();
    armed = false;
  }

  if (gyro.error.z > 180 || gyro.error.z < -180) {
    resetYaw();
  }

  if (NoDataCount > 3) {
    killSwitch = 2;
  }

  if (gyro.error.x > maxAngle || gyro.error.x < (-maxAngle)) {
    if (killAngle) killSwitch = 1;
  }

  if (gyro.error.y > maxAngle || gyro.error.y < (-maxAngle)) {
    if (killAngle) killSwitch = 1;
  }

  if (killSwitch > 0) {
    stopMotors();

    while (killSwitch > 0) {
      tone(BUZZER, 1000, 300);
      led(300);
      delay(2000);

      if (killSwitch == 2 && radio.available()) {
        delay(500);

        if (radio.available()) {
          tone(BUZZER, 1500, 1000);
          led(1000);
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
      tone(BUZZER, 1500, 500);
      led(500);
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
      tone(BUZZER, 1200, 100);
      led(100);
      delay(300);
      tone(BUZZER, 1200, 200);
      led(200);

      cal = gyro.calibrate(1000);

      EEPROM.put(10, static_cast<float>(cal.x));
      EEPROM.put(15, static_cast<float>(cal.y));

      delay(500);

      gyro.setCalibration(cal);
      tone(BUZZER, 2200, 200);
      led(200);

      delay(1000);
      calCount = 0;
    }
  } else {
    calCount = 0;
  }
}

void calculate_battery() {
  real_voltage = analogRead(A0);
  vout = (real_voltage * 5.0f) / 1023.0f;
  vin = vout / (R2 / (R1 + R2));
}

void waitLoop() {
  while (micros() - prevTime < timepi * sec_to_micro) {
    // Busy wait to maintain fixed loop rate
  }
  prevTime = micros();
}

void led(int t) {
  digitalWrite(LED, HIGH);
  delay(t);
  digitalWrite(LED, LOW);
}

void readEEPROM() {
  EEPROM.get(10, cal.x);
  EEPROM.get(15, cal.y);
}

void debugging(bool dBug) {
  if (dBug) {
    dBugging = true;
    Serial.begin(57600);
    hz = 140;
  }
}

void resetYaw() {
  gyro.zeroYaw(true);
  target.z = 0;
}

void Print() {
  Serial.print(F("actual_pressure= "));
  Serial.print(actual_pressure);
  Serial.print('\t');
  Serial.print(F("kalman_velocity_z= "));
  Serial.print(actual_pressure_2);
  Serial.println('\t');
}
