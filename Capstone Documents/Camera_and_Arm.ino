#include "Arduino.h"

#include <Servo.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <SPI.h>  
#include <Pixy.h>
#include <SharpIR.h>

Pixy pixy;
#define ir A0
#define model 1080
SharpIR sharp(ir, 25, 93, model);

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)       
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)
//Select arm
#define AL5D

const float A = 5.75;
const float B = 7.375;
const float D = 7.0;

//Radians to Degrees constant
const float rtod = 57.295779;

//Arm temp pos
float tmpx = 4;
float tmpy = 4;
float tmpz = 90;



// Number of positions to cycle through
const int numPos = 3;
class ServoLoop
{
public:
  ServoLoop(int32_t pgain, int32_t dgain);

  void update(int32_t error);
   
  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_pgain;
  int32_t m_dgain;
};


ServoLoop panLoop(300, 500);
ServoLoop tiltLoop(500, 700);

ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
{
  m_pos = PIXY_RCS_CENTER_POS;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}

void ServoLoop::update(int32_t error)
{
  long int vel;
  char buf[32];
  if (m_prevError!=0x80000000)
  {	
    vel = (error*m_pgain + (error - m_prevError)*m_dgain)>>10;
    //sprintf(buf, "%ld\n", vel);
    //Serial.print(buf);
    m_pos += vel;
    if (m_pos>PIXY_RCS_MAX_POS) 
      m_pos = PIXY_RCS_MAX_POS; 
    else if (m_pos<PIXY_RCS_MIN_POS) 
      m_pos = PIXY_RCS_MIN_POS;
  }
  m_prevError = error;
}

void setup()
{
  Serial.begin(9600);
  
  //set the starting position
  Serial.println("#0 P1500 T500");
  Serial.println("#1 P1500 T500");
  Serial.println("#2 P1500 T500");
  Serial.println("#3 P500 T500");
  Serial.println("#4 P1500 T500");
  pixy.init();
  pinMode (ir, INPUT);
}

//Kinematics to control Arm
int Arm(float x, float y)
{
  float M = sqrt((y*y)+(x*x));
  if(M <= 0)
    return 1;
  float A1 = atan(y/x);
  if(x <= 0)
    return 1;
  float A2 = acos((A*A-B*B+M*M)/((A*2)*M));
  float Elbow = acos((A*A+B*B-M*M)/((A*2)*B));
  float Shoulder = A1 + A2;
  Elbow = Elbow * rtod;
  Shoulder = Shoulder * rtod;
  if((int)Elbow <= 0 || (int)Shoulder <= 0)
    return 1;
  
  //CONVERT ANGLES TO MICROSECONDS
  int elbowServo_us = map(180-Elbow, 0, 180, 900, 2100);
  //Serial.print("\n\nElbow\ndegrees: "); Serial.print(180-Elbow);  Serial.print("\nmicroseconds: "); Serial.print(elbowServo_us);
  
  int shoulderServo_us = map(Shoulder, 0, 180, 900, 2100);
  //Serial.print("\n\nShoulder\ndegrees: "); Serial.print(Shoulder);  Serial.print("\nmicroseconds: "); Serial.print(shoulderServo_us);
  
  //int baseServo_us = map(z, 0, 180, 900, 2100);
  //Serial.print("\n\nBase\ndegrees: "); Serial.print(z);  Serial.print("\nmicroseconds: "); Serial.print(baseServo_us);
  
  //Serial.print("\n\n");  
  //WRITE SERIALLY TO SSC-32U
  //Serial.print("#0 P"); Serial.print(baseServo_us, DEC); Serial.print(" T1000\r");
  Serial.print("#1 P"); Serial.print(shoulderServo_us, DEC); Serial.print(" T1000\r");
  Serial.print("#2 P"); Serial.print(elbowServo_us, DEC); Serial.print(" T1000\r");
  Serial.println("#3 P900 T500");
  Serial.println("#4 P500 T1000");

  delay(1000);
 
  return 0; 
}

void loop()
{ 
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  int32_t panError, tiltError;
  
  blocks = pixy.getBlocks();
  
  if (blocks)
  {
    panError = X_CENTER-pixy.blocks[0].x;
    tiltError = pixy.blocks[0].y-Y_CENTER;
    
    panLoop.update(panError);
    tiltLoop.update(tiltError);
    
    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
    
    i++;
    
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%50==0) 
    {
      int armRotate = map(1000-panLoop.m_pos,0,1000, 900, 2100);
      Serial.print("#0 P"); Serial.print(armRotate, DEC); Serial.print(" T1000\n");
      
      int cameraAngle = map(panLoop.m_pos,0,1000, 0, 180);
      cameraAngle = cameraAngle / rtod;
      float cmDist = sharp.distance();
      float inDist = cmDist * 2.54;
      float height = 16;
      
      float armRadius = sqrt(inDist*inDist + D*D - inDist*D*cos(cameraAngle));
      
      Arm(armRadius, height);
      
      
    }
  }  
}


