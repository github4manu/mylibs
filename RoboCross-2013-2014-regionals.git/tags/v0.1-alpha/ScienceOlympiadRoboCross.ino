/*
If you are participating in the 2013 Science Olympiad RoboCross event, you may NOT under any circumstances copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software.
================================================================================
RoboCross-2013 Arduino Software by Neil Johari is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
View a copy of it here: http://creativecommons.org/licenses/by-nc-sa/3.0/deed.en_US
*/

#include <Servo.h>

#include <Wire.h>
#include <ArduinoNunchuk.h>


#define BAUDRATE 19200
ArduinoNunchuk nunchuk = ArduinoNunchuk();

 
 
Servo frontLeftServo;  
Servo frontRightServo;  
Servo backLeftServo;  
Servo backRightServo;  

void setup() 
{ 
  Serial.begin(BAUDRATE);
  nunchuk.init();
  
  frontLeftServo.attach(9); 
  frontRightServo.attach(3);
  backLeftServo.attach(11);
  backRightServo.attach(10);
} 
 
void loop() 
{ 
  nunchuk.update();
 
  resetServos();

  

  if(nunchuk.analogX < 123 && nunchuk.analogY > 123){ // Q 2
    int xG = map(nunchuk.analogX, 23, 123, 92, 140);
    int yG = map(nunchuk.analogY, 123, 220, 92, 140);
    Serial.print(xG, DEC);
    Serial.print(' ');
    Serial.print(yG, DEC);
    Serial.print(' ');
    int avg = (xG+yG)/2;
    Serial.println(avg, DEC);
    frontLeftServo.write(avg);
    backLeftServo.write(avg);
    frontRightServo.write(140);
    backRightServo.write(140);
  }
  
  if(nunchuk.analogX < 123 && nunchuk.analogY < 121){ // Q 3
    int xG = map(nunchuk.analogX, 23, 123, 20, 89);
    int yG = map(nunchuk.analogY, 25, 121, 20, 89);
    Serial.print(xG, DEC);
    Serial.print(' ');
    Serial.print(yG, DEC);
    Serial.print(' ');
    int avg = (xG+yG)/2;
    Serial.println(avg, DEC);
    frontLeftServo.write(avg);
    backLeftServo.write(avg);
    frontRightServo.write(20);
    backRightServo.write(20);
  }

  if(nunchuk.analogX > 125 && nunchuk.analogY > 123){ // Q 1
    int xG = map(nunchuk.analogX, 125, 228, 20, 89);
    int yG = map(nunchuk.analogY, 123, 220, 20, 89);
    Serial.print(xG, DEC);
    Serial.print(' ');
    Serial.print(yG, DEC);
    Serial.print(' ');
    int avg = (xG+yG)/2;
    Serial.println(avg, DEC);
    frontLeftServo.write(140);
    backLeftServo.write(140);
    frontRightServo.write(avg);
    backRightServo.write(avg);
  }
  
    if(nunchuk.analogX > 125 && nunchuk.analogY < 121){ // Q 4
    int xG = map(nunchuk.analogX, 125, 228, 92, 140);
    int yG = map(nunchuk.analogY, 25, 121, 92, 140 );
    Serial.print(xG, DEC);
    Serial.print(' ');
    Serial.print(yG, DEC);
    Serial.print(' ');
    int avg = (xG+yG)/2;
    Serial.println(avg, DEC);
    frontLeftServo.write(140);
    backLeftServo.write(140);
    frontRightServo.write(avg);
    backRightServo.write(avg);
  }

} 

void resetServos(){
 frontLeftServo.write(91); 
 backLeftServo.write(91);
 frontRightServo.write(91);
 backRightServo.write(91);
}

