#include <EasyKids_LineFollower.h>

/*
<<<<< Command >>>>>

  lineFollowerSetup();

  blackLine();
  whiteLine();

  readSensor();

  edfSetup(); 
  edfSpeed(speed); Speed (1-100)
  edfStop();

  pidLineFollower(Speed, Max_Speed, KP, KD);
  lineFollowerTimer(Speed, Max_Speed, KP, KD, Time(ms)); 
  lineFollowerCross(Speed, Max_Speed, KP, KD); 
  lineFollower90Left(Speed, Max_Speed, KP, KD); 
  lineFollower90Right(Speed, Max_Speed, KP, KD); 
  robotTurnLeft(Speed); 
  robotTurnRight(Speed);
   
*/

void setup() {
  lineFollowerSetup();
  edfSetup(); 
  blackLine();
}

void loop() {

  // readSensor();  // Show Value Sensor via LCD Display

  waitForStart();

  //  ------ Start EDF -------
  edfSpeed(13); // Starting speed
  delay(2000); // delay to start EDF
  
  // ------ Start Robot -------
  lineFollowerTimer(20, 120, 20, 10, 3000); 
  lineFollowerCross(30, 120, 20, 10); 
  robotTurnRight(30); 
  lineFollower90Left(30, 120, 20, 10); 

  // Stop EDF Controller
  edfStop();
  delay(2000);
 
}


