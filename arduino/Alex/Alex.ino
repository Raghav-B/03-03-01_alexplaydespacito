#include <serialize.h>
#include "constants.h"
#include "stdint.h"
#include "packet.h"

typedef enum {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} Tdirection;
volatile Tdirection dir = STOP;

#define COUNTS_PER_REV      195 
#define WHEEL_CIRC          21.5

#define LF                  5   // Left forward pin
#define LR                  6   // Left reverse pin
#define RF                  11  // Right forward pin
#define RR                  10  // Right reverse pin

volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//Variables to help us move the correct distance
volatile unsigned long deltaDist;
volatile unsigned long deltaTicks,targetTicks;
volatile unsigned long newDist;

// Variables for ultrasonic sensor
long frontDuration, frontDistance, backDuration, backDistance;
bool ultrasonicSafety = true;

TResult readPacket(TPacket *packet)
{
    char buffer[PACKET_SIZE];
    int len;
    len = readSerial(buffer);
    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet); 
}

volatile float ratio = 0.73;

void sendStatus()
{
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  statusPacket.params[10] = ratio;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void sendBadPacket()
{
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
}

void sendBadChecksum()
{
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK(){
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;
  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  DDRD  &= 0b11110011;PORTD |= 0b00001100;
}

volatile float error = 0, integral = 0;

const float Kp = -0.002, Ki = -0.0001;

volatile int leftPICount = 0;
volatile int rightPICount = 0;
volatile unsigned long leftDegrees, rightDegrees;

int degreesToTicks(int deg) {
  int dist = deg/360.0 * 31.4;  return (dist*COUNTS_PER_REV/WHEEL_CIRC);
}

void leftISR(){
  leftPICount++;
  if (dir == FORWARD) 
    forwardDist = (unsigned long)((((float) ++leftForwardTicks )/COUNTS_PER_REV)*WHEEL_CIRC);
  else if (dir == BACKWARD)
    reverseDist = (unsigned long)(((float) ++leftReverseTicks ) / COUNTS_PER_REV * WHEEL_CIRC);
  else if (dir == LEFT) 
    leftReverseTicksTurns++;
  else if (dir == RIGHT) 
    leftForwardTicksTurns++;
  if (leftPICount == 50) {
      if (dir == FORWARD|| dir == BACKWARD) {
        error = rightPICount - leftPICount;
        integral +=  error;
        ratio += (error*Kp > 0.01? error*Kp:0.01) + integral*Ki;
     }
     rightPICount = leftPICount = 0;
   }
}

void rightISR(){
  rightPICount++;
  if (dir == FORWARD)
    rightForwardTicks++;// forwardDist = (unsigned long)((float) ++rightForwardTicks  / COUNTS_PER_REV * WHEEL_CIRC);
  else if (dir == BACKWARD)
    rightReverseTicks++;
  else if (dir == LEFT) 
    rightForwardTicksTurns++;
  else if (dir == RIGHT) 
    rightReverseTicksTurns++;
    if (rightPICount == 50) {
      if (dir == FORWARD|| dir == BACKWARD) {
        error = rightPICount - leftPICount;
        integral +=  error;
        ratio += (error*Kp < -0.01? error*Kp:-0.01) + integral*Ki;
     }
     rightPICount = leftPICount = 0;
   }

}

void setupEINT(){
  EICRA = 0b00001010; EIMSK = 0b00000011;
}

ISR(INT0_vect){
  leftISR();
}

ISR(INT1_vect){
  rightISR();
}

void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(57600);
}

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
}

int readSerial(char *buffer)
{
  int count=0;
  while(Serial.available())
    buffer[count++] = Serial.read();
  return count;
}

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

int val = 150;
void forward(float dist, float speed)
{
  dir = FORWARD;
  val = pwmVal(speed);
  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;

  newDist = forwardDist + deltaDist;
  analogWrite(LF, val);
  analogWrite(RF, val*ratio);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
}

void reverse(float dist, float speed)
{
  dir = BACKWARD;
  val = pwmVal(speed);
  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;

  newDist = reverseDist + dist;
  analogWrite(LR, val);
  analogWrite(RR, val*ratio);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
}

void left(float ang, float speed)
{
  dir = LEFT;
  val = pwmVal(speed);

  deltaTicks = degreesToTicks(ang);
  targetTicks = leftReverseTicksTurns + deltaTicks;
  analogWrite(LR, val);
  analogWrite(RF, val*0.77);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
}

void right(float ang, float speed)
{
  dir = RIGHT;
  val = pwmVal(speed);

  deltaTicks = degreesToTicks(ang);
  targetTicks = rightReverseTicksTurns + deltaTicks;
  analogWrite(LF, val);
  analogWrite(RR, val*0.77);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

void forceForward(uint32_t moveTime, float speed)
{
  dir = FORWARD;
  val = pwmVal(speed);
  analogWrite(LF, val);
  analogWrite(RF, val*0.73);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
  delay(moveTime);
  analogWrite(LF, 0);
  analogWrite(RF, 0);  
}

void forceReverse(uint32_t moveTime, float speed)
{
  dir = BACKWARD;
  val = pwmVal(speed);
  analogWrite(LR, val);
  analogWrite(RR, val*0.73);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
  delay(moveTime);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
}

void forceLeft(uint32_t moveTime, float speed)
{
  dir = LEFT;
  val = pwmVal(speed);
  analogWrite(LR, val);
  analogWrite(RF, val*0.77);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
  delay(moveTime);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

void forceRight(uint32_t moveTime, float speed)
{
  dir = RIGHT;
  val = pwmVal(speed);
  analogWrite(LF, val);
  analogWrite(RR, val*0.77);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  delay(moveTime);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{ 
  switch(dir) {
    case FORWARD:
      analogWrite(LF, 0);
      analogWrite(RF, 0);
      analogWrite(LR, 185);
      analogWrite(RR, 200);
      break;
    case BACKWARD:
      analogWrite(LF, 185);
      analogWrite(RF, 200);
      analogWrite(LR, 0);
      analogWrite(RR, 0); break;
    case RIGHT:
      analogWrite(LR, 185); analogWrite(RF,200);
      analogWrite(LF, 0); analogWrite(RR, 0 );  break;
    case LEFT:
      analogWrite(LR, 0); analogWrite(RF,0);
      analogWrite(LF, 185); analogWrite(RR, 200);  break;
  }
  dir = STOP;
  delay(80);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
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
  ratio = 0.73;
  leftPICount = rightPICount = 0;
}

void clearOneCounter(int which)
{
  clearCounters();
}
void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_STOP:
        sendOK();
        forward(0,0);
      break;
    case COMMAND_GET_STATS:
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
      break;
      
    case COMMAND_SAFETY:
        ultrasonicSafety = !ultrasonicSafety;
        TPacket safePacket;
        safePacket.packetType = PACKET_TYPE_RESPONSE;
        if (ultrasonicSafety) safePacket.command = RESP_SAFETY_ON;
        else safePacket.command = RESP_SAFETY_OFF;
        sendResponse(&safePacket);  
      break;

    case COMMAND_FORCE_FORWARD:
        sendOK();
        forceForward((uint32_t) command->params[0], (float) command->params[1]);
      break;
    
    case COMMAND_FORCE_REVERSE:
        sendOK();
        forceReverse((uint32_t) command->params[0], (float) command->params[1]);
      break;
    
    case COMMAND_FORCE_LEFT:
        sendOK();
        forceLeft((uint32_t) command->params[0], (float) command->params[1]);
      break;
    
    case COMMAND_FORCE_RIGHT:
        sendOK();
        forceRight((uint32_t) command->params[0], (float) command->params[1]);
      break;
    
    /*
     * Implement code for other commands here.
     * 
     */
        
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  cli();setupEINT();setupSerial();startSerial();setupMotors();startMotors();enablePullups();initializeState();sei();setupUltrasonic();
  
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void setupUltrasonic() {
  DDRD |= B10000000; // DECLARE PIN 7 OUTPUT
  DDRB |= B010000; // DECLARE PIN 12 OUTPUT
}

void checkDistance() {
  if (dir == FORWARD) {
    PORTB &= B101111; // SET PIN 12 TO LOW (RIGHT TRIGGER)
    delayMicroseconds(5);
    PORTB |= B010000; // SET PIN 12 TO HIGH (RIGHT TRIGGER)
    delayMicroseconds(10);
    PORTB &= B101111; // SET PIN 12 TO LOW (RIGHT TRIGGER)
    DDRB &= B011111; // DECLARE PIN 13 AS INPUT RIGHT ECHO
    frontDuration = pulseIn(13, HIGH);
    frontDistance = (frontDuration * 0.0343) / 2;
    if ( frontDistance < 15 && ultrasonicSafety == true) {
      stop();
    }
  } else if (dir == BACKWARD) {
    PORTD &= B01111111; // SET PIN 7 LOW (LEFT TRIGGER)
    delayMicroseconds(5);
    PORTD |= B10000000; // SET PIN 7 HIGH (LEFT TRIGGER)
    delayMicroseconds(10);
    PORTD &= B01111111; // SET PIN 7 LOW (LEFT TRIGGER)
    DDRB &= B111110; // DECLARE PIN 8 INPUT (LEFT ECHO)
    backDuration = pulseIn(8, HIGH);
    backDistance = (backDuration * 0.0343) / 2;
    if (backDistance < 15 && ultrasonicSafety == true) {
     stop();
    }
  }
}

void loop() {
  TPacket recvPacket; 
  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD){
      sendBadPacket();
    }
    else if(result == PACKET_CHECKSUM_BAD){
        sendBadChecksum();
    } 
  
  //Serial.println(ratio);
  if (deltaDist > 0) {
    if (dir == FORWARD) {
      analogWrite(RF, val*ratio);
      if (forwardDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        
        stop();
      } 
    } else if (dir == BACKWARD) {
      analogWrite(RR, val*ratio);
      if (reverseDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      } 
    } else if (dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  if (deltaTicks > 0) {
    if (dir == LEFT) {
      analogWrite(RF,val*0.77);
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = targetTicks = 0;
        stop();
      }
    } else if (dir == RIGHT) {
      analogWrite(RR,val*0.77);
      if (rightReverseTicksTurns >= targetTicks) {
        deltaTicks = targetTicks = 0;
        stop();
      }
    } else if (dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }

  }

  if (dir == FORWARD || dir == BACKWARD) {
    checkDistance();
  }
  
}
