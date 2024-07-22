#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Servo.h>

#define PWM_L1 5 // Left PWM 1
#define PWM_L2 6 // Left PWM 2

#define PWM_R1 9 // Right PWM 1
#define PWM_R2 10 // Right PWM 2

#define START_SW A6
#define OLED_RESET -1

#define OUT_LINE 16

enum
{
  CENTER,
  RIGHT,
  LEFT
} out_state = CENTER;

const int sensorArr[11] = { 17, 16, 15, 14, 13, 12, 11, 8, 7, 4, 2 };
const int cellSize = 128 / 11;

bool invertedLine = false;
int error_loop_count = 0;
int previous_error = 0;
int error_sum = 0;

Adafruit_SSD1306 display(OLED_RESET);
Servo ESC;

/// Function to set speed of left motor ///
void Motor_L(int speed)
{
  speed = map(speed, -100, 100, -255, 255);

  // Max speed = 255
  if (!speed)
  {
    analogWrite(PWM_L1, 255);
    analogWrite(PWM_L2, 255);
  }
  else if(speed > 0)
  {
    speed = min(speed, 255);
    digitalWrite(PWM_L1, 0);
    digitalWrite(PWM_L2, speed);
  }
  else 
  {
    speed = abs(max(speed, -255));
    digitalWrite(PWM_L1, speed);
    digitalWrite(PWM_L2, 0);
  }
}

/// Function to set speed of right motor ///
void Motor_R(int speed)
{
  speed = map(speed, -100, 100, -255, 255);

  // Max speed = 255
  if (!speed)
  {
    analogWrite(PWM_R1, 255);
    analogWrite(PWM_R2, 255);
  }
  else if(speed > 0)
  {
    speed = min(speed, 255);
    digitalWrite(PWM_R1, 0);
    digitalWrite(PWM_R2, speed);
  }
  else 
  {
    speed = abs(max(speed, -255));
    digitalWrite(PWM_R1, speed);
    digitalWrite(PWM_R2, 0);
  }
}

void edfSetup()
{
  ESC.attach(3);               // Brushless attach
  ESC.writeMicroseconds(1000); // 1000-2500 maxspeed for Brushless
}

void edfSpeed(int speed)
{
  ESC.writeMicroseconds(map(speed, 0, 100, 1000, 2500)); // Map speed for Brushless
}

void edfStop()
{
  ESC.writeMicroseconds(1000); // Stop Brushless
}

void waitForStart()
{
  while(analogRead(START_SW)) {}
}

int sw_Start()
{
  return analogRead(START_SW);
}

void blackLine()
{
  invertedLine = false;
}

void whiteLine()
{
  invertedLine = true;
}

void lineFollowerSetup()
{
  Serial.begin(9600);

  // Sensor pin mode
  for (short int i = 0; i < 11; i++) {
    if (i == 4) {
      pinMode(sensorArr[4], INPUT_PULLUP);
      continue;
    }
    pinMode(sensorArr[i], INPUT);
  }

  // Stop Motors
  Motor_L(0);
  Motor_R(0);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);

  display.setCursor(18, 4);
  display.setTextSize(2);      
  display.print("EasyKids");

  display.setCursor(8, 25);
  display.setTextSize(1);  
  display.print("Line Follower Robot");

  display.display();

}

void readSensor() {
  bool sensorVal = 0;
  int xPos = 0;

  while (1) 
  {
    display.clearDisplay();  
    
    for (short int i = 0; i < 11; i++) 
    {
      sensorVal = digitalRead(sensorArr[i]);
      xPos = cellSize * i;

      Serial.print(sensorVal); // Serial Monitor Output

      if (!sensorVal) 
      {
        display.fillRect(xPos + 4, 0, cellSize - 4, 15, WHITE);
      } else 
      {
        display.drawLine(xPos + 4, 15, xPos + cellSize, 15, WHITE);
      }

      if (i == 9) 
      {
        display.setCursor(xPos + 1, 22);
      } else 
      {
        display.setCursor(xPos + 4, 22); 
      }                             
      display.setTextSize(1);       
      display.setTextColor(WHITE);  
      display.print(i + 1); // OLED Display Output
    }
    Serial.println("");
    display.display();
  }
}

int readError()
{
  bool sensorVal[11] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  int error = 0;

  bool onLine = 0;
  for (short int i = 0; i < 11; i++) 
  {
    sensorVal[i] = invertedLine ? digitalRead(sensorArr[i]) : !digitalRead(sensorArr[i]);
    onLine |= sensorVal[i];
  }

  if (onLine)
  {
   /* Negative left sensor */
    error = sensorArr[4]                   ? -2 : error; // b1
    error = sensorArr[3]                   ? -4 : error; // b3
    error = sensorArr[2]                   ? -6 : error; // b5
    error = sensorArr[1]                   ? -8 : error; // b7
    error = sensorArr[0]                   ? -10 : error; // b9
    error = sensorArr[4] && sensorArr[3]   ? -3 : error; // b1 && b3
    error = sensorArr[3] && sensorArr[2]   ? -5 : error; // b3 && b5
    error = sensorArr[2] && sensorArr[1]   ? -7 : error; // b5 && b7
    error = sensorArr[1] && sensorArr[0]   ? -9 : error; // b7 && b9
    
    
    /* Positive right sensor */
    error = sensorArr[6]                   ? 2 : error; // b2
    error = sensorArr[7]                   ? 4 : error; // b4 
    error = sensorArr[8]                   ? 6 : error; // b6 
    error = sensorArr[9]                   ? 8 : error; // b8
    error = sensorArr[10]                  ? 10 : error; // b10
    error = sensorArr[6] && sensorArr[7]   ? 3 : error; // b2 && b4
    error = sensorArr[7] && sensorArr[8]   ? 5 : error; // b4 && b6
    error = sensorArr[8] && sensorArr[9]   ? 7 : error; // b6 && b8
    error = sensorArr[9] && sensorArr[10]  ? 9 : error; // b8 && b10
    
    
    /* Neutral middle sensor */
    error = sensorArr[5]                   ? 0 : error; // b0
    error = sensorArr[5] && sensorArr[4]   ? -1 : error; // b0 && b1
    error = sensorArr[5] && sensorArr[6]   ? 1 : error; // b0 && b2
    
    out_state = (error <= 4) && (error >= -4)   ? CENTER : out_state; 
    out_state = (error >= 5) && (error <= 10)   ? LEFT : out_state;
    out_state = (error <= -5) && (error >= -10) ? RIGHT: out_state;

    return error;
  }
  else
  {
    return OUT_LINE;
  }
}

void pidLine(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD)
{

  int speed_1 = 0;
  int speed_2 = 0;
  int current_error = readError();

  if (current_error == OUT_LINE)
  {
    switch (out_state)
    {
    case CENTER:
      speed_1 = MED_SPEED;
      speed_2 = MED_SPEED;
      break;
    case LEFT:
      speed_1 = MAX_SPEED;
      speed_2 =  -MAX_SPEED;
      break;
    case RIGHT:
      speed_1 = -MAX_SPEED;
      speed_2 = MAX_SPEED;
      break;
    }
  }
  else
  {
    int output = (KP * current_error) + (KI * error_sum) + (KD * (current_error - previous_error));

    error_loop_count++;
    if (error_loop_count > 350)
    {
      error_loop_count = 0;
      previous_error = current_error;
      error_sum += current_error;
    }

    if (current_error == 0)
    {
      error_sum = 0;
    }

    speed_1 = MED_SPEED + output;
    speed_2 = MED_SPEED - output;
  }

  Motor_L(speed_1);
  Motor_R(speed_2);
}

void lineTimer(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD, long timer)
{
  long timeSince = millis();
  while (millis() - timeSince < timer)
  {
    pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
  }
  Motor_L(0);
  Motor_R(0);
}

void lineCross(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD) // 00011111000
{
  if (invertedLine)
  {
    while(!digitalRead(sensorArr[3]) || !digitalRead(sensorArr[4]) || !digitalRead(sensorArr[5]) || !digitalRead(sensorArr[6]) || !digitalRead(sensorArr[7]))  
    {
      pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
    }
  }
  else
  {
    while(digitalRead(sensorArr[3]) || digitalRead(sensorArr[4]) || digitalRead(sensorArr[5]) || digitalRead(sensorArr[6]) || digitalRead(sensorArr[7]))
    {
      pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
    }
  }

  Motor_L(MED_SPEED);
  Motor_R(MED_SPEED);
  delay(map(MED_SPEED, 0, 100, 100, 0));

  Motor_L(0);
  Motor_R(0);
}

void lineFork(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD) //  00010001000
{
  if (invertedLine)
  {
    while (!digitalRead(sensorArr[3]) || !digitalRead(sensorArr[7]))
    {
      pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
    }
  }
  else
  {
    while (digitalRead(sensorArr[3]) || digitalRead(sensorArr[7]))
    {
      pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
    }
  }

  Motor_L(MED_SPEED);
  Motor_R(MED_SPEED);
  delay(map(MED_SPEED, 0, 100, 100, 0));

  Motor_L(0);
  Motor_R(0);
}

void line90Left(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD) // 00100100000
{
  if (invertedLine)
  {
    while (!digitalRead(sensorArr[2]) || !digitalRead(sensorArr[5])) 
    {
      pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
    }
  }
  else
  {
    while (digitalRead(sensorArr[2]) || digitalRead(sensorArr[5])) 
    {
      pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
    }
  }

  Motor_L(MED_SPEED);
  Motor_R(MED_SPEED);
  delay(map(MED_SPEED, 0, 100, 100, 0));

  Motor_L(0);
  Motor_R(0);
}

void line90Right(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD) // 00000100100
{
  if (invertedLine)
  {
    while (!digitalRead(sensorArr[5]) || !digitalRead(sensorArr[8])) 
    {
      pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
    }
  }
  else
  {
    while (digitalRead(sensorArr[5]) || digitalRead(sensorArr[8])) 
    {
      pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
    }
  }

  Motor_L(MED_SPEED);
  Motor_R(MED_SPEED);
  delay(map(MED_SPEED, 0, 100, 100, 0));

  Motor_L(0);
  Motor_R(0);
}

void lineTurnLeft(int MED_SPEED) // Second or third leftest sensor
{
  Motor_R(MED_SPEED);
  Motor_L(-MED_SPEED);
  delay(50);

  if (invertedLine)
  {
    while (!digitalRead(sensorArr[2]) && !digitalRead(sensorArr[3])) 
    {
      Motor_R(MED_SPEED);
      Motor_L(-MED_SPEED);
    }
  }
  else
  {
    while (digitalRead(sensorArr[2]) && digitalRead(sensorArr[5])) 
    {
      Motor_R(MED_SPEED);
      Motor_L(-MED_SPEED);
    }
  }

  Motor_L(0);
  Motor_R(0);
}

void lineTurnRight(int MED_SPEED) // Second or third rightest sensor
{
  Motor_R(-MED_SPEED);
  Motor_L(MED_SPEED);
  delay(50);
  
  if (invertedLine)
  {
    while (!digitalRead(sensorArr[7]) && !digitalRead(sensorArr[8]))
    {
      Motor_R(-MED_SPEED);
      Motor_L(MED_SPEED);
    }
  }
  else
  {
    while (digitalRead(sensorArr[7]) && digitalRead(sensorArr[8]))
    {
      Motor_R(-MED_SPEED);
      Motor_L(MED_SPEED);
    }
  }

  Motor_L(0);
  Motor_R(0);
}