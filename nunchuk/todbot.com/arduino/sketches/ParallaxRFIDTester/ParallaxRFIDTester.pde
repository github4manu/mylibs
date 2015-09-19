//
//
//
//


#include "AFSoftSerial.h"

#define enablePin 8
#define soutPin   7
#define sinPin    6

AFSoftSerial mySerial =  AFSoftSerial(soutPin, sinPin);

void setup()  {
  pinMode(13, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Goodnight moon!");
  // set the data rate for the SoftwareSerial port
  mySerial.begin(2400);
  digitalWrite( enablePin, LOW);
  
  //mySerial.println("Hello, world?");
}

void loop()                     // run over and over again
{
  if (mySerial.available()) {
      Serial.print((char)mySerial.read());
  }
  if (Serial.available()) {
      mySerial.print((char)Serial.read());
  }
}
