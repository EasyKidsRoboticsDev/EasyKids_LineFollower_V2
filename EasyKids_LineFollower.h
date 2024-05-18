#include <Servo.h>
Servo ESC;

#define OUT_LINE 16
int esc_speed = 1000;
bool invertedLine = false;

enum
{
  HOME,
  STARTING,
  RUN
} state = HOME;

enum
{
  CENTER,
  RIGHT,
  LEFT
} out_state = CENTER;

signed int error_actual = 0;
signed int error_anterior = 0;
int error_sum = 0;
int error = 0;

signed int speed_1 = 0;
signed int speed_2 = 0;

signed int proportional = 0;
signed int derivative = 0;

int sensor1 = A7;
int sensor2 = 4;
int sensor3 = 5;
int sensor4 = 6;
int sensor5 = 7;
int sensor6 = 8;
int sensor7 = 12;
int sensor8 = 14;
int sensor9 = 15;
int sensor10 = 16;
int sensor11 = A6;

void lineFollowerSetup()
{
  Serial.begin(9600);

  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  pinMode(sensor6, INPUT);
  pinMode(sensor7, INPUT);
  pinMode(sensor8, INPUT);
  pinMode(sensor9, INPUT);
  pinMode(sensor10, INPUT);
  pinMode(sensor11, INPUT);

  pinMode(3, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);

  pinMode(2, INPUT_PULLUP);
  // motor L Stop
  digitalWrite(19, HIGH);
  digitalWrite(18, HIGH);
  analogWrite(3, 0);
  // motor R Stop
  digitalWrite(17, HIGH);
  digitalWrite(10, HIGH);
  analogWrite(11, 0);
}

void blackLine()
{
  invertedLine = false;
}

void whiteLine()
{
  invertedLine = true;
}

void edfSetup()
{
  ESC.attach(9);               // Brushless attach
  ESC.writeMicroseconds(1000); // 1000-2500 maxspeed for Brushless
  // esc_speed = map(speed, 0, 100, 1000, 2500);  //1000-2500 maxspeed for Brushless
}

void edfSpeed(int speed)
{
  ESC.writeMicroseconds(map(speed, 0, 100, 1000, 2500)); // 1000-2500 maxspeed for Brushless
}

void edfStop()
{
  ESC.writeMicroseconds(1000); // 1000-2500 maxspeed for Brushless
}

void waitForStart()
{
  while (!digitalRead(2))
  {
  }
}

int sw_Start()
{
  return digitalRead(2);
}

int checkSensor(int pin)
{
  if (analogRead(pin) > 200)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void readSensor()
{
  while (1)
  {
    Serial.print(!(checkSensor(sensor1)));
    Serial.print(!(digitalRead(sensor2)));
    Serial.print(!(digitalRead(sensor3)));
    Serial.print(!(digitalRead(sensor4)));
    Serial.print(!(digitalRead(sensor5)));
    Serial.print(!(digitalRead(sensor6)));
    Serial.print(!(digitalRead(sensor7)));
    Serial.print(!(digitalRead(sensor8)));
    Serial.print(!(digitalRead(sensor9)));
    Serial.print(!(digitalRead(sensor10)));
    Serial.println(!(checkSensor(sensor11)));
    delay(100);
  }
}

signed int Read_error(void)
{
  int b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10;
  signed int error = 0;

  if (invertedLine)
  {
    b0 = digitalRead(sensor6);
    b1 = digitalRead(sensor5);
    b2 = digitalRead(sensor7);
    b3 = digitalRead(sensor4);
    b4 = digitalRead(sensor8);
    b5 = digitalRead(sensor3);
    b6 = digitalRead(sensor9);
    b7 = digitalRead(sensor2);
    b8 = digitalRead(sensor10);
  }
  else
  {
    b0 = !(digitalRead(sensor6));
    b1 = !(digitalRead(sensor5));
    b2 = !(digitalRead(sensor7));
    b3 = !(digitalRead(sensor4));
    b4 = !(digitalRead(sensor8));
    b5 = !(digitalRead(sensor3));
    b6 = !(digitalRead(sensor9));
    b7 = !(digitalRead(sensor2));
    b8 = !(digitalRead(sensor10));
  }

  if (b0 || b1 || b2 || b3 || b4 || b5 || b6 || b7 || b8)
  {

    /*Negative left sensor*/
    error = (b1) ? (0 - 2) : error;
    error = (b3) ? (0 - 4) : error;
    error = (b5) ? (0 - 6) : error;
    error = (b1 && b3) ? (0 - 3) : error;
    error = (b3 && b5) ? (0 - 5) : error;
    error = (b7) ? (0 - 8) : error;
    error = (b5 && b7) ? (0 - 7) : error;

    /*Positive right sensor*/
    error = (b2) ? 2 : error;
    error = (b4) ? 4 : error;
    error = (b6) ? 6 : error;
    error = (b2 && b4) ? 3 : error;
    error = (b4 && b6) ? 5 : error; 
    error = (b8) ? 8 : error;
    error = (b6 && b8) ? 7 : error;

    /*Neutral middle sensor*/
    error = (b0) ? 0 : error;
    error = (b0 && b1) ? (0 - 1) : error;
    error = (b0 && b2) ? 1 : error;

    out_state = ((error <= 4) && (error >= (0 - 4))) ? CENTER : out_state;
    out_state = ((error >= 5) && (error <= 10)) ? LEFT : out_state;
    out_state = ((error <= (0 - 5)) && (error >= (0 - 10))) ? RIGHT : out_state;

    return error;
  }
  else
  {
    return OUT_LINE;
  }
}

/// Function to set speed of right motor///
void Motor_L(signed int speed)
{
  speed = map(speed, -100, 100, -250, 250);

  // Max speed = 250
  if (!speed)
  {
    analogWrite(3, 255);
    digitalWrite(18, HIGH);
    digitalWrite(19, HIGH);
  }
  else
  {
    speed = (speed >= 250) ? 250 : speed;

    if (speed >= 1)
    {
      analogWrite(3, speed);
      digitalWrite(18, HIGH);
      digitalWrite(19, LOW);
    }
    else
    {
      speed *= (0 - 1);
      ////////////////////////
      speed = (speed >= 250) ? 250 : speed;
      ///////////////////////////////////
      analogWrite(3, speed);
      digitalWrite(18, LOW);
      digitalWrite(19, HIGH);
    }
  }
  return;
}

/// Function to set speed of left motor///
void Motor_R(signed int speed)
{
  speed = map(speed, -100, 100, -250, 250);

  // Max speed = 250
  if (!speed)
  {
    analogWrite(11, 255);
    digitalWrite(10, HIGH);
    digitalWrite(17, HIGH);
  }
  else
  {
    speed = (speed >= 250) ? 250 : speed;

    if (speed >= 1)
    {
      analogWrite(11, speed);
      digitalWrite(10, HIGH);
      digitalWrite(17, LOW);
    }
    else
    {
      speed *= (0 - 1);
      ////////////////////////
      speed = (speed >= 250) ? 250 : speed;
      ///////////////////////////////////
      analogWrite(11, speed);
      digitalWrite(10, LOW);
      digitalWrite(17, HIGH);
    }
  }
  return;
}

void pidLine(int MED_SPEED, int max_speed, int KP, int KD)
{
  error_actual = Read_error();

  if (error_actual == OUT_LINE)
  {
    switch (out_state)
    {
    case CENTER:
      speed_1 = MED_SPEED;
      speed_2 = MED_SPEED;
      break;
    case LEFT:
      speed_1 = max_speed;
      speed_2 = (0 - max_speed);
      break;
    case RIGHT:
      speed_1 = (0 - max_speed);
      speed_2 = max_speed;
      break;
    }
  }
  else
  {
    proportional = (KP * error_actual);
    derivative = (KD * (error_actual - error_anterior));

    error_sum++;
    if (error_sum > 350)
    {
      error_anterior = error_actual;
      error_sum = 0;
      error += error_actual;
    }
    if (error_actual == 0)
    {
      error = 0;
    }
    speed_1 = MED_SPEED + (proportional + derivative);
    speed_2 = MED_SPEED - (proportional + derivative);
  }
  Motor_R(speed_2);
  Motor_L(speed_1);
}

void lineTimer(int MED_SPEED, int max_speed, int KP, int KD, long timer)
{
  long timeSince = millis();
  while (millis() - timeSince < timer)
  {
    pidLine(MED_SPEED, max_speed, KP, KD);
  }
  Motor_R(0);
  Motor_L(0);
}

void lineCross(int MED_SPEED, int max_speed, int KP, int KD)
{
  if (invertedLine)
  {
    while(!digitalRead(sensor3) && !digitalRead(sensor4) && !digitalRead(sensor5) && !digitalRead(sensor6) && !digitalRead(sensor7) && !digitalRead(sensor8) && !digitalRead(sensor9)) // 2 Outer left sensors AND 2 Outer right sensors
    {
      pidLine(MED_SPEED, max_speed, KP, KD);
    }
  }
  else
  {
    while(digitalRead(sensor3) && digitalRead(sensor4) && digitalRead(sensor5) && digitalRead(sensor6) && digitalRead(sensor7) && digitalRead(sensor8) && digitalRead(sensor9)) // 2 Outer left sensors AND 2 Outer right sensors
    {
      pidLine(MED_SPEED, max_speed, KP, KD);
    }
  }
  Motor_L(MED_SPEED);
  Motor_R(MED_SPEED);
  delay(map(MED_SPEED, 0, 100, 100, 0));
  Motor_R(0);
  Motor_L(0);
}

void line90Left(int MED_SPEED, int max_speed, int KP, int KD)
{
  if (invertedLine)
  {
    while (!digitalRead(sensor2) || !digitalRead(sensor3) || !digitalRead(sensor4) || !digitalRead(sensor5)) // 3 Left sensors
    {
      pidLine(MED_SPEED, max_speed, KP, KD);
    }
  }
  else
  {
    while (digitalRead(sensor2) || digitalRead(sensor3) || digitalRead(sensor4) || digitalRead(sensor5)) // 3 Left sensors
    {
      pidLine(MED_SPEED, max_speed, KP, KD);
    }
  }
  Motor_L(MED_SPEED);
  Motor_R(MED_SPEED);
  delay(map(MED_SPEED, 0, 100, 100, 0));
  Motor_R(0);
  Motor_L(0);
}

void line90Right(int MED_SPEED, int max_speed, int KP, int KD)
{
  if (invertedLine)
  {
    while (!digitalRead(sensor7) || !digitalRead(sensor8) || !digitalRead(sensor9) || !digitalRead(sensor10)) // 3 Right sensors
    {
      pidLine(MED_SPEED, max_speed, KP, KD);
    }
  }
  else
  {
    while (digitalRead(sensor7) || digitalRead(sensor8) || digitalRead(sensor9) || digitalRead(sensor10)) // 3 Right sensors
    {
      pidLine(MED_SPEED, max_speed, KP, KD);
    }
  }
  Motor_L(MED_SPEED);
  Motor_R(MED_SPEED);
  delay(map(MED_SPEED, 0, 100, 100, 0));
  Motor_R(0);
  Motor_L(0);
}

void lineTurnLeft(int MED_SPEED)
{
  // Motor_R(MED_SPEED);
  // Motor_L(-MED_SPEED);
  // delay(70);

  if (invertedLine)
  {
    while (!digitalRead(sensor2) && !digitalRead(sensor3)) // Leftest sensor
    {
      Motor_R(MED_SPEED);
      Motor_L(-MED_SPEED);
    }
  }
  else
  {
    while (digitalRead(sensor2) && digitalRead(sensor3)) // Leftest sensor
    {
      Motor_R(MED_SPEED);
      Motor_L(-MED_SPEED);
    }
  }
  Motor_R(0);
  Motor_L(0);
}

void lineTurnRight(int MED_SPEED)
{
  // Motor_R(-MED_SPEED);
  // Motor_L(MED_SPEED);
  // delay(70);
  
  if (invertedLine)
  {
    while (!digitalRead(sensor9) && !digitalRead(sensor10)) // Rightest sensor
    {
      Motor_R(-MED_SPEED);
      Motor_L(MED_SPEED);
    }
  }
  else
  {
    while (digitalRead(sensor9) && digitalRead(sensor10)) // Rightest sensor
    {
      Motor_R(-MED_SPEED);
      Motor_L(MED_SPEED);
    }
  }
  Motor_R(0);
  Motor_L(0);
}