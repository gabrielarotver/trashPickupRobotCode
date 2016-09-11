#include <SPI.h>  
#include <Pixy.h>
#include <SharpIR.h>
// This is the main Pixy object 
Pixy pixy;

static int model = 1080; 
int ir = A0;
SharpIR sharp(ir, 25, 93, model);

void setup()
{
  Serial.begin(9600);
  pixy.init();
}

void loop()
{ 
  uint16_t blocks;
  
  // grab blocks!
  blocks = pixy.getBlocks(); 
  
  if (blocks)
  {
    i++;
    
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%50==0)
    {
        int cmDist = sharp.distance();
        float inDist = cmDist / 2.54;
        area = pixy.blocks[0].height * pixy.blocks[0].width;
        
        Serial.print(inDist);
        Serial.print(",");
        Serial.print(pixy.blocks[0].height);
        Serial.print("\n");
    }
  }  
}

