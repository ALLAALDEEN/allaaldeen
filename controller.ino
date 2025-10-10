#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

// Smoothing filters for each axis - improved filtering
Smoothed<float> smoothThrust;
Smoothed<float> smoothXR;
Smoothed<float> smoothXL;
Smoothed<float> smoothYR;

RF24 radio(9, 10);   // CE = D9, CSN = D10

const uint64_t pipe = 0xF0F0F0F0E1LL;

// Calibration values - optimized for better precision
float scaleX = 0.095;     // Reduced from 0.1 for finer control
float calX = -527;
float offsetX = 0;

float scaleY = -0.095;    // Reduced from -0.1 for finer control
float calY = -507;
float offsetY = 0;

float scaleZ = -0.08;     // Reduced from -0.1 for smoother yaw
float calZ = -512;
float offsetZ = 0;

float scaleThrust = 1.45; // Reduced from 1.5 for smoother throttle response
float calThrust = -500;
float offsetThrust = 1300;

// Analog pin assignments
const int XL_pin = 1; // analog pin connected to XL output
const int YL_pin = 0; // analog pin connected to YL output
const int XR_pin = 3; // analog pin connected to XR output
const int YR_pin = 2; // analog pin connected to YR output

// Digital pin assignments
const int SWITCH2_PIN = 2;
const int SWITCH1_PIN = 3;
const int BUTTON1_PIN = 4;
const int BUTTON2_PIN = 5;

int ID = 0;
float xr, yr;
float xl, yl;
float smoothyl, smoothxr, smoothxl, smoothyr;
bool but1, but2, switch1, switch2;

// Function prototypes
void readJoyStick();
void printPackage();
void initializeRadio();

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
  // Initialize digital pins with internal pull-up resistors
  pinMode(SWITCH2_PIN, INPUT_PULLUP);
  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  Serial.begin(115200);

  // Initialize radio
  initializeRadio();

  // Initialize smoothing filters with optimized parameters
  smoothThrust.begin(SMOOTHED_EXPONENTIAL, 15);    // Increased smoothing for thrust
  smoothXR.begin(SMOOTHED_EXPONENTIAL, 8);         // Moderate smoothing for roll
  smoothXL.begin(SMOOTHED_EXPONENTIAL, 12);        // More smoothing for yaw
  smoothYR.begin(SMOOTHED_EXPONENTIAL, 8);         // Moderate smoothing for pitch

  Serial.println("Controller Ready - nRF24L01 Channel 35 (2.435 GHz)");
  
  // Initial calibration check
  delay(1000);
  Serial.println("Place sticks in neutral position and press any button to start...");
  while (digitalRead(BUTTON1_PIN) && digitalRead(BUTTON2_PIN)) {
    delay(100);
  }
  Serial.println("Controller active!");
}

void loop() {
  readJoyStick();

  // Apply calibration and scaling with improved precision
  package.x = (xr + calX) * scaleX + offsetX;
  package.y = (yr + calY) * scaleY + offsetY;
  package.z = (xl + calZ) * scaleZ + offsetZ;
  package.thrust = max(0, (int)((yl + calThrust) * scaleThrust + offsetThrust));
  
  package.id = ID++;
  package.but1 = but1;
  package.but2 = but2;
  package.switch1 = switch1;
  package.switch2 = switch2;

  // Send data with error checking
  bool result = radio.write(&package, sizeof(package));
  if (!result) {
    Serial.println("Transmission failed!");
  }

  printPackage();
  
  // Small delay to prevent overwhelming the receiver
  delay(7);  // ~140Hz update rate to match flight controller
}

void initializeRadio() {
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(35);   // Channel 35 → 2.435 GHz (matches receiver)
  radio.openWritingPipe(pipe);
  radio.stopListening();
  
  // Verify radio initialization
  if (radio.isChipConnected()) {
    Serial.println("nRF24L01 connected successfully");
  } else {
    Serial.println("nRF24L01 connection failed!");
  }
}

void readJoyStick() {
  // Read digital inputs
  but1 = digitalRead(BUTTON1_PIN);
  but2 = digitalRead(BUTTON2_PIN);
  switch1 = digitalRead(SWITCH1_PIN);
  switch2 = digitalRead(SWITCH2_PIN);

  // Read analog inputs with improved processing
  smoothxr = analogRead(XR_pin);
  smoothxl = analogRead(XL_pin);
  smoothyr = 1023 - analogRead(YR_pin);  // Invert Y-axis for proper pitch control
  smoothyl = max(0, analogRead(YL_pin)); // Ensure thrust is never negative

  // Apply smoothing filters
  smoothThrust.add(smoothyl);
  smoothXR.add(smoothxr);
  smoothXL.add(smoothxl);
  smoothYR.add(smoothyr);

  // Get smoothed values
  xr = smoothXR.get();
  xl = smoothXL.get();
  yr = smoothYR.get();
  yl = smoothThrust.get();
}

void printPackage() {
  // Print in a more readable format
  Serial.print("T:");
  Serial.print(package.thrust);
  Serial.print(" X:");
  Serial.print(package.x, 2);
  Serial.print(" Y:");
  Serial.print(package.y, 2);
  Serial.print(" Z:");
  Serial.print(package.z, 2);
  Serial.print(" S1:");
  Serial.print(package.switch1);
  Serial.print(" S2:");
  Serial.print(package.switch2);
  Serial.print(" B1:");
  Serial.print(package.but1);
  Serial.print(" B2:");
  Serial.println(package.but2);
}