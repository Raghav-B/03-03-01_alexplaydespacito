#include <serialize.h>



#include "packet.h"
#include "constants.h"

typedef enum {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} Tdirection;



volatile Tdirection dir = STOP;

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      195 // in theory 192, but 195 was more accurate
                                // over 50 rotations
// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          21.5

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  5   // Left forward pin
#define LR                  6   // Left reverse pin
#define RF                  11  // Right forward pin
#define RR                  10  // Right reverse pin

/*
 *    Alex's State Variables
 */

volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//Variables to help us move the correct distance
volatile unsigned long deltaDist;
volatile unsigned long newDist;
// Set up encoder interrupts



void setupEINT()
{
  EICRA = 0b00001010;
  EIMSK = 0b00000011;
  
}

ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}

// converts [0,100] to [0,255]
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}


volatile float ratio = 0.73;
volatile float error = 0,diff = 0, integral = 0;

const float Kp = -0.002, Ki = -0.0001  , Kd = 0;


volatile int leftCount = 0;
volatile int rightCount = 0;
void rightISR()
{
  rightCount++;
  if (dir == FORWARD)
    rightForwardTicks++;
  else if (dir == BACKWARD)
    rightReverseTicks++;
  else if (dir == LEFT) 
    rightForwardTicksTurns++;
  else if (dir == RIGHT) 
    rightReverseTicksTurns++;

  if (rightCount == 50) {
    if (dir != STOP) {
    diff = (rightCount - leftCount) - error;
    error = rightCount - leftCount;
    integral +=  error;
    ratio += (error*Kp < -0.01? error*Kp:-0.01) + integral*Ki;
    Serial.print(leftCount); Serial.print(" ");
    Serial.print(rightCount); Serial.print(" "); 
    
    Serial.print(error*Kp); Serial.print(" "); 
    Serial.print(integral); Serial.print(" ");
    Serial.println(diff*Kd); Serial.print(" ");  

  
    //Serial.println(cumulative_error); Serial.print(" "); 

   }
     rightCount = leftCount = 0;
   
  }

  

}

void leftISR()
{
  leftCount++;
  if (dir == FORWARD) 
    forwardDist = (unsigned long)((float) ++leftForwardTicks  / COUNTS_PER_REV * WHEEL_CIRC);
  else if (dir == BACKWARD)
    reverseDist = (unsigned long)((float) ++leftReverseTicks  / COUNTS_PER_REV * WHEEL_CIRC);
  else if (dir == LEFT) 
    leftReverseTicksTurns++;
  else if (dir == RIGHT) 
    leftForwardTicksTurns++;

   if (leftCount == 50) {
    if (dir != STOP) {
    diff = (rightCount - leftCount) - error;
    error = rightCount - leftCount;
    integral +=  error;
    ratio += (error*Kp > 0.01? error*Kp:0.01) + integral*Ki;
    Serial.print(leftCount); Serial.print(" ");
    Serial.print(rightCount); Serial.print(" ");
    
    Serial.print(error*Kp); Serial.print(" ");
    Serial.print(integral); Serial.print(" ");
    Serial.println(diff*Kd); Serial.print(" "); 

    
    }

    rightCount = leftCount = 0;
  }
}

// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  DDRD  &= 0b11110011;
  PORTD |= 0b00001100;
}


void forward(float dist, float speed)
{
  dir = FORWARD;
  int val = pwmVal(speed);
  /*if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;*/
   integral = diff = error = 0;
  analogWrite(LR, 0);
  analogWrite(RR, 0);
  for (int i = 0; i < 80; i++) {
    analogWrite(LF, val);
    analogWrite(RF, val*ratio);
   delay(10);
  }
  
 
  dir = STOP;
  analogWrite(LF, 0);
  analogWrite(RF, 0);
  analogWrite(LR, 200);
  analogWrite(RR, 200);
  delay(25);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
  
  
}

void clearCounters()
{
  
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
   cli();
  setupEINT();
  enablePullups();
  clearCounters();
    sei();

  forward(20,50);
  delay(1000);
  forward(20,50);
  delay(1000);
  forward(20,50);


}

void loop() {
  // put your main code here, to run repeatedly:

  

}
