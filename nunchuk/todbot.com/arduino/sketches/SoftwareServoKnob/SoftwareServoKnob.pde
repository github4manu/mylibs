/*
 * Yet another of those "read the knob, move the servo" sketches.
 * But this time, uses the SoftwareServo library (aka "Servo") available
 * from: http://www.arduino.cc/playground/ComponentLib/Servo
 * 
 * 2009, Tod E. Kurt, http://todbot.com/blog/
 */

#include "SoftwareServo.h"

SoftwareServo servo1;

int loopcnt = 0;

void setup()
{
  servo1.attach(2);  // pin 2 is our servo
  Serial.begin(19200);
  Serial.println("SoftwareServoKnob!");
}

void loop()
{
  int val = analogRead(0);  // read the knob

  val = map(val, 0,1023, 0,180); // convert to 0-180 degrees
  servo1.write(val);
  
  if( loopcnt++ == 500 ) {
    Serial.print("angle: ");
    Serial.println( val );
    loopcnt = 0;
  }

  SoftwareServo::refresh();
  delay(1);
}
