#include <EasyKids_LineFollower.h>

void setup() {
  lineFollowerSetup();

  // --- Choose Line Type --------
  blackLine();
  //whiteLine();
}

void loop() {
  readSensor(); // Show Value Sensor via LCD Display
}
