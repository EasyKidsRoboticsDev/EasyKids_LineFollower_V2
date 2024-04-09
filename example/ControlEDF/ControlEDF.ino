#include <EasyKids_LineFollower.h>

void setup() {
  lineFollowerSetup();
  edfSetup(); // EDF setup
}

void loop() {
  waitForStart();
  edfSpeed(13); // Set EDF speed (1-100)
  delay(2000);
  edfStop(); // Stop EDF
  delay(2000);
}
