#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

Smoothed<float> degerexpo;
Smoothed<float> degerxrexpo;
Smoothed<float> degerxlexpo;
Smoothed<float> degeryrexpo;

RF24 radio(9, 10);   // CE = D9, CSN = D10

const uint64_t pipe = 0xF0F0F0F0E1LL;

// Calibration and scaling constants
float scaleX = 0.1;
float calX = -527;
float offsetX = 0;

float scaleY = -0.1;
float calY = -507;
float offsetY = 0;

float scaleZ = -0.1;
float calZ = -512;
float offsetZ = 0;

float scaleThrust = 1.5;
float calThrust = -500;
float offsetThrust = 1300;

// Joystick pins
const int XL_pin = 1; // analog pin connected to XL output
const int YL_pin = 0; // analog pin connected to YL output
const int XR_pin = 3; // analog pin connected to XR output
const int YR_pin = 2; // analog pin connected to YR output

int ID = 0;
float xr, yr;
float xl, yl;
float smoothyl, smoothxr, smoothxl, smoothyr;
bool but1, but2, switch1, switch2;

void readJoyStick();
void printPackage();

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
  radio.setChannel(35);   // Must match receiver channel
  radio.openWritingPipe(pipe);
  radio.stopListening();

  // FIX: Initialize smoothing filters with proper factor
  degerexpo.begin(SMOOTHED_EXPONENTIAL, 10);
  degerxrexpo.begin(SMOOTHED_EXPONENTIAL, 10);
  degerxlexpo.begin(SMOOTHED_EXPONENTIAL, 10);
  degeryrexpo.begin(SMOOTHED_EXPONENTIAL, 10);

  Serial.println("Controller Ready - nRF24L01 Channel 35 (2.435 GHz)");
}

void loop() {
  readJoyStick();

  // Apply calibration and scaling
  package.x = (xr + calX) * scaleX + offsetX;
  package.y = (yr + calY) * scaleY + offsetY;
  package.z = (xl + calZ) * scaleZ + offsetZ;
  package.thrust = (yl + calThrust) * scaleThrust + offsetThrust;
  
  // FIX: Constrain thrust to safe range
  package.thrust = constrain(package.thrust, 1000, 2000);
  
  package.id = ID++;
  package.but1 = but1;
  package.but2 = but2;
  package.switch1 = switch1;
  package.switch2 = switch2;

  // Send data
  bool success = radio.write(&package, sizeof(package));
  
  // FIX: Optional - monitor transmission success
  if (!success && Serial) {
    // Transmission failed - could add retry logic or warning
  }
  
  printPackage();
  
  // FIX: Small delay for stability (adjust for desired update rate)
  delayMicroseconds(100);
}

void readJoyStick() {
  but1 = digitalRead(4);
  but2 = digitalRead(5);
  switch1 = digitalRead(3);
  switch2 = digitalRead(2);

  // Read analog values
  smoothxr = analogRead(XR_pin);
  smoothxl = analogRead(XL_pin);
  smoothyr = 1023 - analogRead(YR_pin);  // FIX: Changed 1013 to 1023 for proper inversion
  smoothyl = max(analogRead(YL_pin), 0);

  // Add to smoothing filters
  degerexpo.add(smoothyl);
  degerxrexpo.add(smoothxr);
  degerxlexpo.add(smoothxl);
  degeryrexpo.add(smoothyr);

  // Get smoothed values
  xr = degerxrexpo.get();
  xl = degerxlexpo.get();
  yr = degeryrexpo.get();
  yl = degerexpo.get();
}

void printPackage() {
  Serial.print("Thrust: ");
  Serial.print(package.thrust);
  Serial.print("\tZ: ");
  Serial.print(package.z);
  Serial.print("\tX: ");
  Serial.print(package.x);
  Serial.print("\tY: ");
  Serial.print(package.y);
  Serial.print("\tID: ");
  Serial.println(package.id);
}
