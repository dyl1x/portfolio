#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"
//minmax_leg-joint value
#define min_11 120;
#define max_11 480;
#define min_21 120;
#define max_21 490;
#define min_31 100;
#define max_31 400;
#define min_41 120;
#define max_41 450;

#define min_12 110;
#define max_12 450;
#define min_22 120;
#define max_22 500;
#define min_32 130;
#define max_32 440;
#define min_42 120;
#define max_42 450;

#define min_13 110;
#define max_13 450;
#define min_23 120;
#define max_23 480;
#define min_33 110;
#define max_33 440;
#define min_43 110;
#define max_43 450;

int servoval[2][12] = {
  {120,120,100,120,110,120,130,120,110,120,110,110}, //min
  {450,490,400,450,450,500,440,450,450,480,440,450} //max
};




// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

int pulselen = 200;
int input = 0;
unsigned int intval = 0;
char incoming;
bool startup = false;

void loop() 
{
  if (Serial.available()>0)
  {
    intval = 0;
    while (1)
    {
      incoming  = Serial.read();
      if (incoming == '\n') break;
      if (incoming == -1) continue;
      intval *= 10;
      intval = ((incoming - 48)+intval); 
    }
    Serial.println(intval);
    pulselen = intval;
  }
    int no = 3;
    pwm.setPWM(no, 0, pulselen);
    pwm.setPWM(no+4, 0, pulselen);
    pwm.setPWM(no+8, 0, pulselen);
}

int calvalue(int deg,int servonum)
{
  
  int smin = servoval[0][servonum];
  int smax = servoval[1][servonum];
  
  //value = (((max-min)/180)*deg)+min

  float val = ((smax-smin)/ (float)180)*deg;
  val = val + smin;
  return val;
}

void pos_initialize()
{
  for (int x = 0; x < 12; x++)
  {
    int pulselen = calvalue(90, x);
    Serial.print("len");
    Serial.println(pulselen);
    pwm.setPWM(x,0, pulselen);
    delay(1000);
  }
  
}
