/**
 * ScaryShiftServoEyeballs.pde --
 *  Get two eyeballs, hook a servo to each one, 
 *  then mount that on a 'head' and connect a servo to that.
 *  This sketch will make the eyes and head "look around" in a shifty fashion.
 *  Add sensors to make it look at things!
 *
 * 2009, Tod E. Kurt, http://todbot.com/blog/
 *
 */

#include <Servo.h> 
 
Servo servoHead; 
Servo servoEyeL; 
Servo servoEyeR;

int posHead;
int posEye;
int posHdest, posEdest;  
int wait;
int rate = 2;

void setup() 
{ 
  servoHead.attach(5); 
  servoEyeL.attach(6); 
  servoEyeR.attach(7); 

  posHdest = posHead = 90;
  posEdest = posEye = 90;
} 
 
void loop() 
{ 
    if( wait == 0 ) {
        posHdest = random(70,110);  // move had between 70-110 degrees
        posEdest = random(40,140);  // move eyes between 40-140 degrees
        wait = random(150,450);     // between 1.5 secs & 4.5 secs
    }
    else {
        posHead = slew_servo( posHead, posHdest, rate);
        posEye  = slew_servo( posEye,  posEdest, rate);
        wait--;
    }
 
    servoHead.write( posHead );
    servoEyeL.write( posEye  );
    servoEyeR.write( posEye  );    

    delay(10);
}

// return new pos from current pos, dest and rate
int slew_servo( int pos, int dest, int rate) 
{
    int diff = pos - dest;
    if( diff == 0 ) return pos;
    else if( diff > -rate ) pos++;
    else if( diff <  rate ) pos--;
    if(      diff > 0 ) pos -= rate;
    else if( diff < 0 ) pos += rate;
    return pos;
}

