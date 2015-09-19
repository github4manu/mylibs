//
// SerLCDTest -- 
//
// 2008, Tod E. Kurt, http://todbot.com/blog/
//

#include "AFSoftSerial.h"

#define lcdoutPin 2
#define lcdinPin 3  // not really

AFSoftSerial lcdSerial =  AFSoftSerial(lcdinPin, lcdoutPin);

char str[9];

void setup()
{
  Serial.begin(19200);
  Serial.println("SerLCDTest");

  lcdSerial.begin(9600);
  lcdSerial.print("hello there");

  strcpy(str,"--------");
  delay(1000);
  
}

byte i,inc=1;
void loop() 
{ 
  lcdSerial.print(0xFE,BYTE  ); // clear display
  lcdSerial.print(0x01,BYTE); // clear display
  lcdSerial.print(str);
  strcpy(str,"--------");
  str[i] = '*'; 
  //str[i-inc] = '-';
  i+=inc;
  if(i==7) inc=-1;
  if(i==0) inc=+1;
  delay(200);
}
