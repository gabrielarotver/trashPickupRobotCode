//===================================================================================
//  Pixy Guided Rover will try to follow Objects
//  Portions of this code are derived from the Pixy CMUcam5 pantilt example code. 
//
//  Robot with 4 geared DC motors controlled by Sabertooth 2x10, µCU = ATmega328P-PU
//  Pixy tracking object, communicates via SPI with µCU
//  Pixy mounted on pan/tilt mount controlled by 2 GS-9018 servos
//  Pixy data:
//  block 1 X = 0 object detected on the left
//  block 1 X = 319 object detected on the right
//  block 1 Y = 0 object detected top
//  block 1 Y = 199 object detected bottom
//===================================================================================

#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>
 
#define X_CENTER          ((PIXY_MAX_X - PIXY_MIN_X) / 2)           // 159 is center of 0-319
#define Y_CENTER          ((PIXY_MAX_Y - PIXY_MIN_Y) / 2)           // 99 is center of 0-199
#define RCS_MIN_POS       PIXY_RCS_MIN_POS                          // 0
#define RCS_MAX_POS       PIXY_RCS_MAX_POS                          // 1000
#define RCS_CENTER_POS    ((RCS_MAX_POS - RCS_MIN_POS) / 2)         // 500
#define SCAN_INCREMENT    ((RCS_MAX_POS - RCS_MIN_POS) / 100)       // 10
 
#define MIN_PIXEL_AREA    1                                         // Smallest Pixel Area Possible
#define MAX_PIXEL_AREA    (PIXY_MAX_X + 1) * (PIXY_MAX_Y + 1)       // 320 * 200 = 64000 Pixels Squared

#define STAND_STILL_AREA   8649                                     // Stop moving at when Object's Pixel Area is this big
#define HIGHEST_SPEED_VAL  STAND_STILL_AREA - 1                     // 8648
#define LOWEST_SPEED_VAL   STAND_STILL_AREA - MAX_PIXEL_AREA     // -(64000 - 8649) = -55351

#define MOTOR_CENTER  1500              // No Movement Pulse Value
#define MAX_FWD_SPEED 1300              // Maximum Forward/Turn Right Speed
#define MAX_REV_SPEED 1700              // Maximum Backward/Turn Left Speed 

//----------------------------------------------
// Servo Loop Class
// A Proportional/Derivative feedback loop for
// pan/tilt servo tracking of blocks.
// (Based on Pixy CMUcam5 pan/tilt example code)
//----------------------------------------------
class ServoLoop
{
public:
   ServoLoop(int32_t proportionalGain, int32_t derivativeGain);
 
   void update(int32_t error);
 
   int32_t m_pos;
   int32_t m_prevError;
   int32_t m_proportionalGain;
   int32_t m_derivativeGain;
};
 
// ServoLoop Constructor
ServoLoop::ServoLoop(int32_t proportionalGain, int32_t derivativeGain)
{
   m_pos = RCS_CENTER_POS;
   m_proportionalGain = proportionalGain;
   m_derivativeGain = derivativeGain;
   m_prevError = 0x80000000L;
}
 
// ServoLoop Update 
// Calculates new output based on the measured
// error and the current state.
void ServoLoop::update(int32_t error)
{
   long int velocity;
   if ( m_prevError != 0x80000000 )
   {
      velocity = (error*m_proportionalGain + (error - m_prevError)*m_derivativeGain)>>10;
 
      // 500 is Center
      // Position Range (500, 1000] is left for Pan or Up for Tilt Servos
      // Position Range [0, 500) is right for Pan or Down for Tilt Servos 
      m_pos = constrain( m_pos + velocity, RCS_MIN_POS, RCS_MAX_POS );
   }
   m_prevError = error;
}
// End Servo Loop Class
//---------------------------------------
 
Pixy pixy;                      // Declare the camera object
 
ServoLoop panLoop(300, 500);    // Servo loop for pan
ServoLoop tiltLoop(500, 700);   // Servo loop for tilt

const int leftMotorPin  = 4;    // PD5 - PWM  Drive Forward/Backward or Left Channel (S2)
const int rightMotorPin = 2;    // PB2 - PWM  Turn Left/Right or Right Channel (S1)
 
Servo leftMotor;
Servo rightMotor;
 
//---------------------------------------
// Setup - runs once at startup
//---------------------------------------
void setup()
{
   //Serial.begin(115200);
   //Serial.print("Starting...\n");

   // Initialize SPI Communication
   pixy.init();

   leftMotor.attach(leftMotorPin, MAX_FWD_SPEED, MAX_REV_SPEED);
   rightMotor.attach(rightMotorPin, MAX_FWD_SPEED, MAX_REV_SPEED);
}
 
//------------------------------------------
// Main loop - runs continuously after setup
//------------------------------------------
void loop()
{
   static uint32_t lastBlockTime = 0;
   static uint16_t blocks;
   static int trackedBlock;
   blocks = pixy.getBlocks();
 
   // If we have blocks in sight, track and follow them
   if (blocks)
   {
      trackedBlock = TrackBlock(blocks);
      FollowBlock(trackedBlock);
      lastBlockTime = millis();
   }  
   // If no blocks have been detected for 100ms, stop and scan for blocks
   else if (millis() - lastBlockTime > 100)
   {
      leftMotor.writeMicroseconds(MOTOR_CENTER);
      rightMotor.writeMicroseconds(MOTOR_CENTER);
      ScanForBlocks();
   }
}

 
//------------------------------------------------
// Track blocks via the Pixy pan/tilt mechanism
// (based in part on Pixy CMUcam5 pantilt example)
//------------------------------------------------
int TrackBlock(int blockCount)
{
   static int oldSignature = 0;
   static int32_t panError;
   static int32_t tiltError;
   static int trackedBlock;
   static long maxSize;
   static long newSize;
   static int i;
 
   // These variables need be reset every for every call
   maxSize = 0;
   trackedBlock = 0;
 
   // Track the closest/largest signature object
   for ( i = 0; i < blockCount; i++ )
   {
      // If no signature has been set, track the object with the greatest size,
      // Otherwise track the object of greatest size for a set signature
      // The signature of the tracked object will be remembered.
      // It can only be reset, by restarting the program.
      if ((oldSignature == 0) || (pixy.blocks[i].signature == oldSignature))
      {
         newSize = pixy.blocks[i].height * pixy.blocks[i].width;
         if (newSize > maxSize)
         {
            trackedBlock = i;
            maxSize = newSize;
         }
      }
   }
 
   // Object is to left if > 0, or to right if < 0     Range: [-159,159]
   panError = X_CENTER - pixy.blocks[trackedBlock].x;  // 159 - x
   // Object is up if < 0 or down if > 0               Range: [-99,99]
   tiltError = pixy.blocks[trackedBlock].y - Y_CENTER; // y - 99
 
   // Update the position of the pan/tilt servos
   panLoop.update(panError);
   tiltLoop.update(tiltError);
 
   // Set the new position of the pan/tilt servos
   pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
 
   // Save the signature of the tracked object
   oldSignature = pixy.blocks[trackedBlock].signature;
   return trackedBlock;
}
 
//---------------------------------------------
// Get the Rover to Follow blocks
//
// This code makes the robot base turn and move
// to follow the pan/tilt tracking of the head.
//---------------------------------------------
void FollowBlock( int trackedBlock )
{
   static int32_t size = STAND_STILL_AREA;
   static int32_t followError;
   static int forwardSpeed;
   static int32_t differential;
   static int leftSpeed;
   static int rightSpeed;
   //static int i = 0;

   // Ranges from -500 to 500, Object is to right if > 0, or to left if < 0 
   followError = RCS_CENTER_POS - panLoop.m_pos;  // How far off-center are we looking now?

   // Take the rolling average of the object's area over 8 samples.
   size -= size/8;
   size += (pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height)/8;

   // Forward speed decreases as we approach the object (size is larger)
   forwardSpeed = map(size, MIN_PIXEL_AREA, MAX_PIXEL_AREA, HIGHEST_SPEED_VAL, LOWEST_SPEED_VAL);
   
   /*
   if (i % 50 == 0)
   {
      Serial.print("Follow Error: ");
      Serial.println(followError);
      Serial.print("Object Size: ");
      Serial.println(size);
      Serial.print("Raw Forward Speed: ");
      Serial.println(forwardSpeed);
   }
   */
   
   // Constrain the forward speed
   forwardSpeed = constrain(forwardSpeed, -HIGHEST_SPEED_VAL/2, HIGHEST_SPEED_VAL);
   
   // Steering differential is proportional to the error times the forward speed
   if ( forwardSpeed > 0)
   {
      differential = (followError + (followError * forwardSpeed))>>8; //-16894 to 16894
   }
   else
   {
      differential = (followError - (followError * forwardSpeed))>>8; //-8447 to 8447
   }

   // Adjust the left and right speeds by the steering differential.
   if ( size < STAND_STILL_AREA )
   {
      leftSpeed = constrain(forwardSpeed + differential, -HIGHEST_SPEED_VAL, HIGHEST_SPEED_VAL);
      rightSpeed = constrain(forwardSpeed - differential, -HIGHEST_SPEED_VAL, HIGHEST_SPEED_VAL);
   }
   // Rotate the rover if the pan servo position is more than 12 degrees off center.
   else if ( abs(followError) > 100 )
   {
      leftSpeed = constrain(differential, -HIGHEST_SPEED_VAL, HIGHEST_SPEED_VAL);
      rightSpeed = -leftSpeed;
   }
   // Move the rover backwards
   else
   {
      leftSpeed = forwardSpeed;
      rightSpeed = forwardSpeed;
   }
   
   /*
   if ( i % 50 == 0 )
   {
      Serial.print("Constrained Forward Speed: ");
      Serial.println(forwardSpeed);
      Serial.print("Steering Differential: ");
      Serial.println(differential);
      Serial.print("Left Speed: ");
      Serial.println(leftSpeed);
      Serial.print("Right Speed: ");
      Serial.println(rightSpeed);
      Serial.println();
      i = 0;
   }
   i++;
   */
   
   // Map Motor Speeds To Servo Pulse Width in Microseconds
   leftSpeed = map(leftSpeed, -HIGHEST_SPEED_VAL, HIGHEST_SPEED_VAL, MAX_REV_SPEED, MAX_FWD_SPEED);
   rightSpeed = map(rightSpeed, -HIGHEST_SPEED_VAL, HIGHEST_SPEED_VAL, MAX_REV_SPEED, MAX_FWD_SPEED);
 
   // Set the motor speeds
   leftMotor.writeMicroseconds(leftSpeed);
   rightMotor.writeMicroseconds(rightSpeed);
}
 
//---------------------------------------
// Random search for blocks
//
// This code pans back and forth at random
// until a block is detected
//---------------------------------------
void ScanForBlocks()
{
   static int scanIncrement = SCAN_INCREMENT;
   static uint32_t lastMove = 0;

   // Gradually adjust camera pan after 20 milliseconds have passed
   if (millis() - lastMove > 20)
   {
      lastMove = millis();
      // Pan the camera all the way to leftmost or rightmost position, before switching
      panLoop.m_pos += scanIncrement;
      // If left or right panning bounds have been reached, adjust the camera's tilt and turn the rover
      if ( (panLoop.m_pos >= RCS_MAX_POS) || (panLoop.m_pos <= RCS_MIN_POS) )
      {
         // Randomly adjust the vertical pan between slightly below horizon level and upwards
         tiltLoop.m_pos = random( RCS_MAX_POS * 0.2, RCS_MAX_POS * 0.6 ); // 200 to 600 (scan up to slightly below horizontal level)
         scanIncrement = -scanIncrement;
         // Search to the left, turn the rover counter-clockwise
         if (scanIncrement < 0)
         {
            leftMotor.writeMicroseconds(1600);
            rightMotor.writeMicroseconds(1400);
         }
         // Search to the right, turn the rover clockwise
         else
         {
            leftMotor.writeMicroseconds(1400);
            rightMotor.writeMicroseconds(1600);
         }
         // The rover will turn for a random duration of 250-500 milliseconds
         delay(random(250, 500));
      }
 
      // Set the new position of the pan/tilt servos
      pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
   }
}
