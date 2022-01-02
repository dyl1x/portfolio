#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50

bool _pos = 0;  // 0=sit, 1 = stand

//pulse len: 150-600
void setup() 
{
   pwm.begin();
   pwm.setOscillatorFrequency(2500000);
   pwm.setPWMFreq(SERVO_FREQ);
   delay(10);
   Serial.begin(9600);
   
}

int incoming = 0;
unsigned int intval = 0;
int val = 0;

void loop() 
{
    pwm.setPWM(4, 0, 550);
 
}



void pos_sit()
{
  if (_pos == 1)
  {
    
  }
}

void pos_stand()
{
  if (_pos == 0)
  {
    
  }
}
