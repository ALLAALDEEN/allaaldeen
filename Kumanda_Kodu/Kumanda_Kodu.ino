#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

Smoothed<float> degerexpo;
Smoothed<float> degerxrexpo;
Smoothed<float> degerxlexpo;
Smoothed<float> degeryrexpo;

RF24 radio(9, 10);

const uint64_t pipe = 0xF0F0F0F0E1LL;

float scaleX = 0.1f;
float calX = -527;
float offsetX = 0;

float scaleY = -0.1f;
float calY = -507;
float offsetY = 0;

float scaleZ = -0.1f;
float calZ = -512;
float offsetZ = 0;

float scaleThrust = 1.5f;
float calThrust = -500;
float offsetThrust = 1300;

const int XL_pin = 1;
const int YL_pin = 0;

const int XR_pin = 3;
const int YR_pin = 2;

int ID = 0;
float xr = 0;
float yr = 0;
float xl = 0;
float yl = 0;
float smoothyl = 0;
float smoothxr = 0;
float smoothxl = 0;
float smoothyr = 0;
bool but1 = true;
bool but2 = true;
bool switch1 = true;
bool switch2 = true;

struct Package
{
  int thrust = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  int id = 0;
  bool but1 = true;
  bool but2 = true;
  bool switch1 = true;
  bool switch2 = true;
};

Package package{};

void readJoyStick();
void printPackage();

void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);

  Serial.begin(57600);
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(76);
  radio.openWritingPipe(pipe);
  radio.stopListening();

  degerexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  degerxrexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  degerxlexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  degeryrexpo.begin(SMOOTHED_EXPONENTIAL, 1);
}

void loop() {
  readJoyStick();

  package.x = (xr + calX) * scaleX + offsetX;
  package.y = (yr + calY) * scaleY + offsetY;
  package.z = (xl + calZ) * scaleZ + offsetZ;
  package.thrust = static_cast<int>((yl + calThrust) * scaleThrust + offsetThrust);
  package.id = ID++;
  package.but1 = but1;
  package.but2 = but2;
  package.switch1 = switch1;
  package.switch2 = switch2;

  radio.write(&package, sizeof(package));
  printPackage();
}

void readJoyStick()
{
  but1 = digitalRead(4);
  but2 = digitalRead(5);
  switch1 = digitalRead(3);
  switch2 = digitalRead(2);

  smoothxr = analogRead(XR_pin);
  smoothxl = analogRead(XL_pin);
  smoothyr = 1013 - analogRead(YR_pin);
  smoothyl = max(analogRead(YL_pin), 0);

  degerexpo.add(smoothyl);
  degerxrexpo.add(smoothxr);
  degerxlexpo.add(smoothxl);
  degeryrexpo.add(smoothyr);

  xr = degerxrexpo.get();
  xl = degerxlexpo.get();
  yr = degeryrexpo.get();
  yl = degerexpo.get();
}

void printPackage()
{
  Serial.print(package.thrust);
  Serial.print("\t");
  Serial.print(package.z);
  Serial.print("\t");
  Serial.print(package.x);
  Serial.print("\t");
  Serial.println(package.y);
}
