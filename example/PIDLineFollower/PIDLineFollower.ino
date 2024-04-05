#include <EasyKids_LineFollower.h>

/*
<<<<< Command >>>>>
  readSensor();

  pidLineFollower(Speed, KP, KD);
  lineFollowerTimer(Speed, KP, KD, Time(ms)); 
  lineFollowerCross(Speed, KP, KD); 
  lineFollower90Left(Speed, KP, KD); 
  lineFollower90Right(Speed, KP, KD); 
  robotTurnLeft(Speed); 
  robotTurnRight(Speed);
   
*/

void setup() {
  lineFollowerSetup();
  escSetup();
  setESCSpeed(50);
}

void loop() {

  // readSensor();  //Show Value

  lineFollowerTimer(20, 1.0, 0.1,1000); 
  lineFollowerCross(30, 1.0, 0.1); 
  lineFollowerTimer(35, 1.0, 1.0, 5000); //lineFollowTime (Speed, KP, KD, Time(ms));
  robotTurnLeft(30); // lineTurnLeft(Speed);
 
}


