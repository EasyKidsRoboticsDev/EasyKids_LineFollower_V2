#include "EasyKids_LineFollower_V2.h"
#include "includes/Adafruit_GFX_Library/Adafruit_GFX.h"
#include "includes/Adafruit_SSD1306/Adafruit_SSD1306.h"
#include "includes/ServoTimer2/ServoTimer2.h"
#include <Arduino.h>

static const int PWM_L1 = 5; // Left PWM 1
static const int PWM_L2 = 6; // Left PWM 2

static const int PWM_R1 = 9; // Right PWM 1
static const int PWM_R2 = 10; // Right PWM 2

static const int EDF_PIN = 3;
static const int START_SW = A6;
static const int OLED_RESET = -1;

static const int OUT_LINE = 16;

static enum OutState 
{
  CENTER,
  RIGHT,
  LEFT
} out_state = CENTER;

static const int sensorArr[11] = { 17, 16, 15, 14, 13, 12, 11, 8, 7, 4, 2 };
static const int cellSize = 128 / 11;
static bool invertedLine = false;

static Adafruit_SSD1306 display(OLED_RESET);
static ServoTimer2 edf;

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
  analogWrite(PWM_L1, 255);
  analogWrite(PWM_L2, 255);
  analogWrite(PWM_R1, 255);
  analogWrite(PWM_R2, 255);

  // Default display
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

void edfSetup()
{
  edf.attach(EDF_PIN); // Brushless attach
  edf.write(1000); // 1000-2500 maxspeed for Brushless
}

void edfSpeed(int speed)
{
  edf.write(map(speed, 0, 100, 1000, 2500)); // Map speed for Brushless
}

void edfStop()
{
  edf.write(1000); // Stop Brushless
}

void Motor_L(int speed)
{
  speed = map(speed, -100, 100, -255, 255); // Max speed = 255
  if (!speed)
  {
    analogWrite(PWM_L1, 255);
    analogWrite(PWM_L2, 255);
  }
  else if(speed > 0)
  {
    speed = min(speed, 255);
    analogWrite(PWM_L1, 0);
    analogWrite(PWM_L2, speed);
  }
  else 
  {
    speed = abs(max(speed, -255));
    analogWrite(PWM_L1, speed);
    analogWrite(PWM_L2, 0);
  }
}

void Motor_R(int speed)
{
  speed = map(speed, -100, 100, -255, 255); // Max speed = 255
  if (!speed)
  {
    analogWrite(PWM_R1, 255);
    analogWrite(PWM_R2, 255);
  }
  else if(speed > 0)
  {
    speed = min(speed, 255);
    analogWrite(PWM_R1, 0);
    analogWrite(PWM_R2, speed);
  }
  else 
  {
    speed = abs(max(speed, -255));
    analogWrite(PWM_R1, speed);
    analogWrite(PWM_R2, 0);
  }
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
      } 
      else 
      {
        display.drawLine(xPos + 4, 15, xPos + cellSize, 15, WHITE);
      }

      if (i == 9) 
      {
        display.setCursor(xPos + 1, 22);
      } 
      else 
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

static int readError()
{
  bool sensorVal[11] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  bool onLine = 0;
  int error = 0;
  for (short int i = 0; i < 11; i++) 
  {
    sensorVal[i] = invertedLine ? digitalRead(sensorArr[i]) : !digitalRead(sensorArr[i]);
    onLine |= sensorVal[i];
  }

  if (!onLine)
  {
    return OUT_LINE;
  }
  
  /* Negative left sensor */
  error = sensorVal[4]                   ? -2 : error; // b1
  error = sensorVal[3]                   ? -4 : error; // b3
  error = sensorVal[2]                   ? -6 : error; // b5
  error = sensorVal[1]                   ? -8 : error; // b7
  error = sensorVal[0]                   ? -10 : error; // b9
  error = sensorVal[4] && sensorVal[3]   ? -3 : error; // b1 && b3
  error = sensorVal[3] && sensorVal[2]   ? -5 : error; // b3 && b5
  error = sensorVal[2] && sensorVal[1]   ? -7 : error; // b5 && b7
  error = sensorVal[1] && sensorVal[0]   ? -9 : error; // b7 && b9
  
  
  /* Positive right sensor */
  error = sensorVal[6]                   ? 2 : error; // b2
  error = sensorVal[7]                   ? 4 : error; // b4 
  error = sensorVal[8]                   ? 6 : error; // b6 
  error = sensorVal[9]                   ? 8 : error; // b8
  error = sensorVal[10]                  ? 10 : error; // b10
  error = sensorVal[6] && sensorVal[7]   ? 3 : error; // b2 && b4
  error = sensorVal[7] && sensorVal[8]   ? 5 : error; // b4 && b6
  error = sensorVal[8] && sensorVal[9]   ? 7 : error; // b6 && b8
  error = sensorVal[9] && sensorVal[10]  ? 9 : error; // b8 && b10
  
  
  /* Neutral middle sensor */
  error = sensorVal[5]                   ? 0 : error; // b0
  error = sensorVal[5] && sensorVal[4]   ? -1 : error; // b0 && b1
  error = sensorVal[5] && sensorVal[6]   ? 1 : error; // b0 && b2
  
  out_state = (error <= 4) && (error >= -4)   ? CENTER : out_state; 
  out_state = (error >= 5) && (error <= 10)   ? LEFT : out_state;
  out_state = (error <= -5) && (error >= -10) ? RIGHT: out_state;

  return error;
}

void pidLine(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD)
{
  static int error_loop_count = 0;
  static int previous_error = 0;
  static int error_sum = 0;
  
  int speed_1 = 0;
  int speed_2 = 0;

  // Upscaling the coefficient values
  KP *= 10;
  KI *= 10;
  KD *= 10;

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
      default:
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
  int ss3 = invertedLine ? !digitalRead(sensorArr[3]) : digitalRead(sensorArr[3]); // Sensor 3
  int ss4 = invertedLine ? !digitalRead(sensorArr[4]) : digitalRead(sensorArr[4]); // Sensor 4
  int ss5 = invertedLine ? !digitalRead(sensorArr[5]) : digitalRead(sensorArr[5]); // Sensor 5
  int ss6 = invertedLine ? !digitalRead(sensorArr[6]) : digitalRead(sensorArr[6]); // Sensor 6
  int ss7 = invertedLine ? !digitalRead(sensorArr[7]) : digitalRead(sensorArr[7]); // Sensor 7
  while(ss3 || ss4 || ss5 || ss6 || ss7)
  {
    ss3 = invertedLine ? !digitalRead(sensorArr[3]) : digitalRead(sensorArr[3]); // Sensor 3
    ss4 = invertedLine ? !digitalRead(sensorArr[4]) : digitalRead(sensorArr[4]); // Sensor 4
    ss5 = invertedLine ? !digitalRead(sensorArr[5]) : digitalRead(sensorArr[5]); // Sensor 5
    ss6 = invertedLine ? !digitalRead(sensorArr[6]) : digitalRead(sensorArr[6]); // Sensor 6
    ss7 = invertedLine ? !digitalRead(sensorArr[7]) : digitalRead(sensorArr[7]); // Sensor 7
    pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
  }

  Motor_L(MED_SPEED);
  Motor_R(MED_SPEED);
  delay(map(MED_SPEED, 0, 100, 100, 0));

  Motor_L(0);
  Motor_R(0);
}

void line90Left(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD) // 00100100000
{
  int ss2 = invertedLine ? !digitalRead(sensorArr[2]) : digitalRead(sensorArr[2]); // Sensor 2
  int ss5 = invertedLine ? !digitalRead(sensorArr[5]) : digitalRead(sensorArr[5]); // Sensor 5
  while(ss2 || ss5)
  {
    ss2 = invertedLine ? !digitalRead(sensorArr[2]) : digitalRead(sensorArr[2]); // Sensor 2
    ss5 = invertedLine ? !digitalRead(sensorArr[5]) : digitalRead(sensorArr[5]); // Sensor 5
    pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
  }

  Motor_L(MED_SPEED);
  Motor_R(MED_SPEED);
  delay(map(MED_SPEED, 0, 100, 100, 0));

  Motor_L(0);
  Motor_R(0);
}

void line90Right(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD) // 00000100100
{
  int ss5 = invertedLine ? !digitalRead(sensorArr[5]) : digitalRead(sensorArr[5]); // Sensor 5
  int ss8 = invertedLine ? !digitalRead(sensorArr[8]) : digitalRead(sensorArr[8]); // Sensor 8
  while(ss5 || ss8)
  {
    ss5 = invertedLine ? !digitalRead(sensorArr[5]) : digitalRead(sensorArr[5]); // Sensor 5
    ss8 = invertedLine ? !digitalRead(sensorArr[8]) : digitalRead(sensorArr[8]); // Sensor 8
    pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
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

  int ss2 = invertedLine ? !digitalRead(sensorArr[2]) : digitalRead(sensorArr[2]); // Sensor 2
  int ss3 = invertedLine ? !digitalRead(sensorArr[3]) : digitalRead(sensorArr[3]); // Sensor 3
  while(ss2 || ss3)
  {
    ss2 = invertedLine ? !digitalRead(sensorArr[2]) : digitalRead(sensorArr[2]); // Sensor 2
    ss3 = invertedLine ? !digitalRead(sensorArr[3]) : digitalRead(sensorArr[3]); // Sensor 3
    Motor_R(MED_SPEED);
    Motor_L(-MED_SPEED);
  }
  Motor_L(0);
  Motor_R(0);
}

void lineTurnRight(int MED_SPEED) // Second or third rightest sensor
{
  Motor_R(-MED_SPEED);
  Motor_L(MED_SPEED);
  delay(50);
  
  int ss7 = invertedLine ? !digitalRead(sensorArr[7]) : digitalRead(sensorArr[7]); // Sensor 7
  int ss8 = invertedLine ? !digitalRead(sensorArr[8]) : digitalRead(sensorArr[8]); // Sensor 8
  while(ss7 || ss8)
  {
    ss7 = invertedLine ? !digitalRead(sensorArr[7]) : digitalRead(sensorArr[7]); // Sensor 7
    ss8 = invertedLine ? !digitalRead(sensorArr[8]) : digitalRead(sensorArr[8]); // Sensor 8
    Motor_R(-MED_SPEED);
    Motor_L(MED_SPEED);
  }
  Motor_L(0);
  Motor_R(0);
}
