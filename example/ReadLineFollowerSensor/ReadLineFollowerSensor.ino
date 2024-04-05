#include <EasyKids_LineFollower.h>

void setup() {
  lineFollowerSetup();
  escSetup();
  setESCSpeed(50);
}

void loop() {
  readSensor(); // Show Value Sensor via LCD Display
}
