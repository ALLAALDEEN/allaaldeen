#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "Smoothed.h"

Smoothed<float> thrustExpo;
Smoothed<float> xrExpo;
Smoothed<float> xlExpo;
Smoothed<float> yrExpo;

RF24 radio(9, 10);   // CE = D9, CSN = D10
const uint64_t pipe = 0xF0F0F0F0E1LL;

// Calibration/scales (tune for your hardware)
float scaleX =  0.1f;  float calX = -527; float offsetX = 0;
float scaleY = -0.1f;  float calY = -507; float offsetY = 0;
float scaleZ = -0.1f;  float calZ = -512; float offsetZ = 0;
float scaleThrust = 1.5f; float calThrust = -500; float offsetThrust = 1300;

const int XL_pin = A1; // left X
const int YL_pin = A0; // left Y (thrust)
const int XR_pin = A3; // right X
const int YR_pin = A2; // right Y

int ID = 0;
float xr, yr, xl, yl;
float smoothyl, smoothxr, smoothxl, smoothyr;
bool but1, but2, switch1, switch2;

struct Package {
  int thrust = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  int id = 0;
  bool but1 = 1;
  bool but2 = 1;
  bool switch1 = 1;
  bool switch2 = 1;
};

Package package;

void readJoyStick();
void printPackage();

void setup() {
  pinMode(2, INPUT_PULLUP); // Switch 2
  pinMode(3, INPUT_PULLUP); // Switch 1
  pinMode(4, INPUT_PULLUP); // Button 1
  pinMode(5, INPUT_PULLUP); // Button 2

  Serial.begin(115200);

  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(90); // must match receiver
  radio.openWritingPipe(pipe);
  radio.stopListening();

  // Use 10% exponential smoothing for lower latency
  thrustExpo.begin(SMOOTHED_EXPONENTIAL, 10);
  xrExpo.begin(SMOOTHED_EXPONENTIAL, 10);
  xlExpo.begin(SMOOTHED_EXPONENTIAL, 10);
  yrExpo.begin(SMOOTHED_EXPONENTIAL, 10);

  Serial.println("Controller Ready - nRF24L01 Channel 90");
}

void loop() {
  readJoyStick();

  package.x =       (xr + calX) * scaleX + offsetX;
  package.y =       (yr + calY) * scaleY + offsetY;
  package.z =       (xl + calZ) * scaleZ + offsetZ;
  package.thrust =  (int)((yl + calThrust) * scaleThrust + offsetThrust);
  package.id = ID++;
  package.but1 = but1;
  package.but2 = but2;
  package.switch1 = switch1;
  package.switch2 = switch2;

  radio.write(&package, sizeof(package));
  printPackage();
}

void readJoyStick() {
  but1 = digitalRead(4);
  but2 = digitalRead(5);
  switch1 = digitalRead(3);
  switch2 = digitalRead(2);

  smoothxr = analogRead(XR_pin);
  smoothxl = analogRead(XL_pin);
  smoothyr = 1013 - analogRead(YR_pin);
  smoothyl = max(analogRead(YL_pin), 0);

  thrustExpo.add(smoothyl);
  xrExpo.add(smoothxr);
  xlExpo.add(smoothxl);
  yrExpo.add(smoothyr);

  xr = xrExpo.get();
  xl = xlExpo.get();
  yr = yrExpo.get();
  yl = thrustExpo.get();
}

void printPackage() {
  Serial.print(package.thrust);  Serial.print("\t");
  Serial.print(package.z);       Serial.print("\t");
  Serial.print(package.x);       Serial.print("\t");
  Serial.println(package.y);
}
