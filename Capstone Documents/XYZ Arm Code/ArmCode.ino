#include "Arduino.h"
#include <Servo.h>
#include <math.h>

//arm dimension from shoulder to elbow (in inches)
const float A = 5.75;
//arm dimension from elbow to wrist (in inches)
const float B = 7.375;

//Radians to Degrees constant
const float rtod = 57.295779;

//temporary values for x, y, and z
float tmpx = 4;
float tmpy = 4;
float tmpz = 90;

// Number of positions to cycle through
const int numPos = 3;

// XYZ Sample Coordinates
float posListXYZWa[][3] ={ {3,2,0}, {6,2,75}, {10,0,140} };

void setup()
{
  //Set baud rate to 9600
  Serial.begin(9600);
  
  //set the starting position
  Serial.println("#0 P1500 T500");
  Serial.println("#1 P1500 T500");
  Serial.println("#2 P1500 T500");
  Serial.println("#3 P500 T500");
  Serial.println("#4 P1500 T500");
}

//Kinematics to control Arm
int Arm(float x, float y, float z){
  float M = sqrt((y*y)+(x*x)); //hypotenuse
  if(M <= 0) //hypotenuse shouldn't be negative, return error value
    return 1;
  float A1 = atan(y/x); 
  if(x <= 0) //X cannot be zero or equations will give error
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
  Serial.print("\n\nElbow\ndegrees: "); Serial.print(180-Elbow);  Serial.print("\nmicroseconds: "); Serial.print(elbowServo_us);
  int shoulderServo_us = map(Shoulder, 0, 180, 900, 2100);
  Serial.print("\n\nShoulder\ndegrees: "); Serial.print(Shoulder);  Serial.print("\nmicroseconds: "); Serial.print(shoulderServo_us);
  int baseServo_us = map(z, 0, 180, 900, 2100);
  Serial.print("\n\nBase\ndegrees: "); Serial.print(z);  Serial.print("\nmicroseconds: "); Serial.print(baseServo_us); Serial.print("\n\n");  
  
  //WRITE SERIALLY TO SSC-32U
  Serial.print("#0 P"); Serial.print(baseServo_us, DEC); Serial.print(" T1000\r");
  Serial.print("#1 P"); Serial.print(shoulderServo_us, DEC); Serial.print(" T1000\r");
  Serial.print("#2 P"); Serial.print(elbowServo_us, DEC); Serial.print(" T1000\r");
  Serial.println("#3 P900 T500");
  Serial.println("#4 P500 T1000");

  delay(1000);
  return 0; 
}

void pickUp(int xValue){
  if(xValue == 3){
    Serial.println("#1 P1300 T1000"); //shoulder forward
    delay(2000);
    Serial.println("#2 P1915 T1000"); //elbow down
    delay(3000); 
  }
  else if (xValue == 6)
  {
    Serial.println("#2 P1550 T1000"); //elbow up
    Serial.println("#3 P760 T1000"); //lower wrist a little
    delay(2000);
    Serial.println("#1 P1100 T1000"); //shoulder forward
    delay(3000);
  }
  else if(xValue == 10) 
  {
    Serial.println("#2 P950 T1000"); //elbow up
    Serial.println("#3 P500 T1000"); //lower wrist a little
    delay(2000);
    Serial.println("#1 P720 T1000"); //shoulder forward
    delay(3000);
  }
  Serial.println("#4 P1500 T1000"); //close grip
  delay(2000);
  Serial.println("#1 P1500 T1000"); //shoulder at middle position
  Serial.println("#2 P1500 T1000"); //elbow at middle position
  delay(3000);
  Serial.println("#3 P900 T500"); //put wrist in standard position
  Serial.println("#0 P2100 T1000"); //move all the way to the right
  delay(3000);
  Serial.println("#1 P1300 T1000"); //shoulder forward
  Serial.println("#2 P1700 T1000"); //elbow down
  delay(2000);
  Serial.println("#4 P500 T1000"); // release object
  delay(2000);
}


void loop()
{
  // Example - Follow set of positions
  for(int i=0; i<numPos; i++)
  {
    // Set positions from array
    tmpx = posListXYZWa[i][0];
    tmpy = posListXYZWa[i][1];
    tmpz = posListXYZWa[i][2];
    
    // Move arm
    Arm(tmpx, tmpy, tmpz);
    delay(1000);
    pickUp(tmpx);
    delay(1000);
  }
}
