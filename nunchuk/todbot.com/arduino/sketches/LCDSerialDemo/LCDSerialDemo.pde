//
// LCDSerialDemo -- demonstrate LCDSerial library for Sparkfun LCDs
//
// Demonstrates how to quickly update the Sparkfun Serial LCD
// using positional commands instead of relying on the Serial LCD's 
// "wrap-around" printing capability (which turns out to be much slower)
// It uses two 16-byte "frame buffers" that contain the characters to write
// to the LCD.
//
// This demo displays two lines: the first lnie has an asterisk "ball" 
// that bounces between two square-bracket "posts" and the second line is
// the current value of millis(). 
// E.g.:
//  .----------------.  
// 0|[-------*------]|  
// 1|23700           |  
//  '----------------'  
//
// 2008, Tod E. Kurt, http://todbot.com/
// 
//


#include "LCDSerial.h"

#include <stdlib.h>
#include <string.h>

#define lcdoutPin 7

// "frame buffer" for LCD
#define lcdNumChars 16
char line1[lcdNumChars+1];
char line2[lcdNumChars+1];

uint8_t i;

LCDSerial lcdSerial =  LCDSerial(lcdoutPin);

void setup() 
{
    Serial.begin(19200);
    Serial.println("LCDSerialDemo");

    lcdSerial.begin(9600);   // don't change! must be 9600
    lcdSerial.clearScreen();
    lcdSerial.print("LCDSerialDemo");

    // clear out the "frame buffer"
    memset( line1, ' ', lcdNumChars);
    memset( line2, ' ', lcdNumChars);

    //             0123456789012345
    strcpy(line1, "[--------------]");
    delay(2000);
}

uint8_t ball = 1;
int8_t ballinc = 1;
unsigned long lasttime;

void loop()
{
    unsigned long t = millis();

    // scribble in the frame buffer
    if( (t - lasttime) > 175 ) {
        lasttime = t;
        strcpy(line1, "[--------------]");
        line1[ball] = '*';
        ball+= ballinc;
        if( ball == 14 ) ballinc=-1;
        if( ball == 1 ) ballinc = 1;
    }
    ///memcpy( line1, "hello", 5);
    ultoa( t, line2, 10);

    // get any of those bad line terminators out of thre
    for( i=0;i<lcdNumChars;i++) {
        if( line1[i]==0 ) line1[i]=' ';
        if( line2[i]==0 ) line2[i]=' ';
    }

    // dump the frame buffer to display
    lcdSerial.gotoLineOne();
    lcdSerial.print(line1);
    lcdSerial.gotoLineTwo();
    lcdSerial.print(line2);

    delay(1);
}

/*
// This is really slow.  I think because of how Sparkfun LCDs parse strings
void loop()
{
    lcdSerial.clearScreen();
    delay(1000);
    lcdSerial.gotoLineOne();
    delay(100);
    lcdSerial.print("hello");
    delay(100);
    lcdSerial.gotoLineTwo();
    delay(100);
    lcdSerial.print( millis()/1000 );
    delay(100);
}
*/
