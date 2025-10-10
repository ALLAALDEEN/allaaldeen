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

// Optimized scaling factors for better control
float scaleX = 0.08;      // Reduced for smoother control
float calX = -527;
float offsetX = 0;

float scaleY = -0.08;     // Reduced for smoother control
float calY = -507;
float offsetY = 0;

float scaleZ = -0.08;     // Reduced for smoother yaw control
float calZ = -512;
float offsetZ = 0;

float scaleThrust = 1.2;  // Optimized for better throttle response
float calThrust = -500;
float offsetThrust = 1300;

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

  // Match the receiver channel
  radio.setChannel(35);   // Channel 35 → 2.490 GHz

  radio.openWritingPipe(pipe);
  radio.stopListening();

  // Initialize smoothing with optimized parameters
  degerexpo.begin(SMOOTHED_EXPONENTIAL, 2);      // Increased smoothing for throttle
  degerxrexpo.begin(SMOOTHED_EXPONENTIAL, 1);    // Less smoothing for roll
  degerxlexpo.begin(SMOOTHED_EXPONENTIAL, 1);    // Less smoothing for yaw
  degeryrexpo.begin(SMOOTHED_EXPONENTIAL, 1);    // Less smoothing for pitch

  Serial.println("Controller Ready - nRF24L01 Channel 35");
}

void loop() {
  readJoyStick();

  // Apply scaling with improved deadzone handling
  package.x = (xr + calX) * scaleX + offsetX;
  package.y = (yr + calY) * scaleY + offsetY;
  package.z = (xl + calZ) * scaleZ + offsetZ;
  package.thrust = (yl + calThrust) * scaleThrust + offsetThrust;
  package.id = ID++;
  package.but1 = but1;
  package.but2 = but2;
  package.switch1 = switch1;
  package.switch2 = switch2;

  // Constrain values for safety
  package.x = constrain(package.x, -1.0, 1.0);
  package.y = constrain(package.y, -1.0, 1.0);
  package.z = constrain(package.z, -1.0, 1.0);
  package.thrust = constrain(package.thrust, 1000, 2000);

  radio.write(&package, sizeof(package));
  printPackage();
  
  // Small delay for stability
  delay(5);
}

void readJoyStick() {
  but1 = digitalRead(4);
  but2 = digitalRead(5);
  switch1 = digitalRead(3);
  switch2 = digitalRead(2);

  // Read analog values with improved mapping
  smoothxr = analogRead(XR_pin);
  smoothxl = analogRead(XL_pin);
  smoothyr = 1013 - analogRead(YR_pin);
  smoothyl = max(analogRead(YL_pin), 0);

  // Apply smoothing
  degerexpo.add(smoothyl);
  degerxrexpo.add(smoothxr);
  degerxlexpo.add(smoothxl);
  degeryrexpo.add(smoothyr);

  xr = degerxrexpo.get();
  xl = degerxlexpo.get();
  yr = degeryrexpo.get();
  yl = degerexpo.get();
}

void printPackage() {
  if (Serial.available() > 0) {
    Serial.print("Thrust: ");
    Serial.print(package.thrust);
    Serial.print(" | Roll: ");
    Serial.print(package.x);
    Serial.print(" | Pitch: ");
    Serial.print(package.y);
    Serial.print(" | Yaw: ");
    Serial.print(package.z);
    Serial.print(" | B1: ");
    Serial.print(package.but1);
    Serial.print(" | B2: ");
    Serial.print(package.but2);
    Serial.print(" | S1: ");
    Serial.print(package.switch1);
    Serial.print(" | S2: ");
    Serial.println(package.switch2);
  }
}