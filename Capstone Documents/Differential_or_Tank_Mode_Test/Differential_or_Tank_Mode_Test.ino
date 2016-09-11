/* 
 * The Differential or Tank Mode Test
 * Translated/Fixed/Improved for BotBoarduino - Dary Cabrera
 */

/* 
 * This program tests the motion of the rover using the A, B, and C buttons on
 * the BotBoarduino.
 * The "A" button controls the right channel throttle.
 * The "C" button controls the left channel throttle.
 * The "B" button resets the speed and direction of both channels.
 * Pressing the A button increments the right channel to full forward, then decrements to full
 * reverse in 10% increments.
 * Pressing the C button increments the Left channel to full forward, then decrements to full
 * reverse in 10% increments.
 * Pressing the B button will reset the speed and direction of both left and right channels.

 * The Sabertooth motor controller has two modes of operation. The default is the mixer mode.
 * This is where you have a Throttle for forward and reverse speed control, and a
 * Steering control for turning. In this mode the left channel is throttle and the right channel is steering. 
 * This mode requires the Sabertooth's DIP Switch 1 to be flipped to the "On" or "Mixing Mode" position.
 * The alternative mode of operation is independent channel control.
 * This is where the left and right channels are controlled independently to steer like a tank.
 * Note: The Sabertooth's red Error LED will light to indicate overheating or current limit. The
 * green Status1 LED will glow dimly when power is applied, and brightly when it's receiving
 * pulses from the microcontroller. The green Status2 LED will flash out the detected number of
 * lithium cells when lithium mode is enabled.
 
 * BotBoarduino Jumpers
 * Speaker enable
 * VS to VL
 * A, B, C, button enable
 * AX 0-3 power bus to VL
 */

#include <Servo.h>

#define STOP_VALUE        1500         // stop value
#define MAX_FORWARD_SPEED 1250         // full forward value
#define MAX_REVERSE_SPEED 1750         // full reverse value

#define FORWARD 0                      // forward motion
#define REVERSE 1                      // backwards motion

const int starting_motion = FORWARD;   // Start Left and Right channel increments in forward motion

const int left_motor_pin  = 4;         // PD5 - PWM  Drive Forward/Backward or Left Channel (S2)
const int right_motor_pin = 2;         // PB2 - PWM  Turn Left/Right or Right Channel (S1)

const int sound_pin = 5;               // PC5 - Speaker Pin

const int a_button = 7;                // PD7 - RED     LED    Forward/Backwards Drive
const int b_button = 8;                // PB0 - GREEN   LED
const int c_button = 9;                // PB1 - YELLOW  LED    Turn Drive

Servo left_motor;
Servo right_motor;

int left_speed   = STOP_VALUE;         // Set Left channel to start at rest
int right_speed  = STOP_VALUE;         // Set Right channel to start at rest
char l_direction = starting_motion;    // Set Left channel starting direction
char r_direction = starting_motion;    // set RIght channel starting direction

int increment = abs((STOP_VALUE - MAX_FORWARD_SPEED)/10);  // The difference must be evenly divisible

void debug_output()
{
  Serial.print( "; Left Speed:  " );
  Serial.print( left_speed );
  Serial.print( "; Right Speed:  " );
  Serial.print( right_speed );
  Serial.println();
}

void setup()
{
  Serial.begin( 115200 );
  pinMode( a_button, INPUT );
  pinMode( b_button, INPUT );
  pinMode( c_button, INPUT );
  
  left_motor.attach( left_motor_pin, MAX_FORWARD_SPEED, MAX_REVERSE_SPEED );
  right_motor.attach( right_motor_pin, MAX_FORWARD_SPEED, MAX_REVERSE_SPEED );
  
  // Generate start-up sound sequence -  pin, frequency(Hz), duration(ms)
  tone(sound_pin, 659, 200);  // E5 note
  delay(250);
  tone(sound_pin, 659, 200);  // E5 note
  delay(350);
  tone(sound_pin, 659, 200);  // E5 note
  delay(275);
  tone(sound_pin, 523, 200);  // C5 note
  delay(275);
  tone(sound_pin, 659, 200);  // E5 note
  delay(400);
  tone(sound_pin, 784, 200);  // G5 note
  delay(200);

  Serial.println( "Setup be complete. " );
}

void loop()
{
  // A button increments then decrements right_speed variable by 10% each press.
  if ( LOW == digitalRead(a_button) ) {
    right_adjust();
    debug_output();
  }
  // B button resets both speed variables to stopped.
  else if ( LOW == digitalRead(b_button) ) {
    reset_both();
    debug_output();
  }
   // C button increments then decrements left_speed variable by 10% each press.
  else if ( LOW == digitalRead(c_button ) ) {
    left_adjust();
    debug_output();
  }
  
  left_motor.writeMicroseconds( left_speed );  // Left Sabertooth channel.
  right_motor.writeMicroseconds( right_speed );  // Right Sabertooth channel.
}

void right_adjust()
{
  while ( LOW == digitalRead(a_button) )  // Hold and release
   tone(sound_pin, 880, 200);   // A5 note.
  delay(10);
  if ( r_direction == FORWARD )
    r_down();
  else
    r_up();
}

void r_up()
{
  right_speed = min( right_speed + increment, MAX_REVERSE_SPEED );
  if ( right_speed == MAX_REVERSE_SPEED )
    r_direction = FORWARD;
}

void r_down()
{
  right_speed = max( right_speed - increment, MAX_FORWARD_SPEED);
  if ( right_speed == MAX_FORWARD_SPEED )
    r_direction = REVERSE;
}

void left_adjust()
{
  while( LOW == digitalRead(c_button) )  // Hold and release
    tone( sound_pin, 1047, 200 );   // C6 note
  delay(10);
  if ( l_direction == FORWARD )
    l_down();
  else
    l_up();
}

void l_up()
{
  left_speed = min( left_speed + increment, MAX_REVERSE_SPEED );
  if ( left_speed == MAX_REVERSE_SPEED )
    l_direction = FORWARD;
}

void l_down()
{
  left_speed = max( left_speed - increment, MAX_FORWARD_SPEED );
  if ( left_speed == MAX_FORWARD_SPEED )
    l_direction = REVERSE;
}

void reset_both()
{
  while( LOW == digitalRead(b_button) )  // Hold and release
    tone( sound_pin, 988, 200 );   // B5 note
  right_speed = STOP_VALUE;
  left_speed =  STOP_VALUE;
  r_direction = FORWARD;
  l_direction = FORWARD;
}

