/*
 * BlinkMCylon -- 
 *
 *
 *
 * 2008 Tod E. Kurt, http://thingm.com/
 *
 */

#include "Wire.h"
#include "BlinkM_funcs.h"

int ledPin = 13;

#define num_blinkms 13
#define blinkm_start_addr 10

//#define max_t2  1500
#define max_t2  10
#define fadespeed 7

//byte blinkm_addrs[num_blinkms] = {
//  10,11,12,13,14,15,16,
byte curr_blinkm = 0;
int incdec = 1;  // only +1 or -1
byte t1;         // t1 runs from 0-255,0-255,... rolls over
int t2 = max_t2; // t2 is the number of t1s to wait before doing cylong thing
byte debug = 1;

//-- knob data ---------------
#define num_knobs 3
#define red_knob 2
#define grn_knob 1
#define blu_knob 0

typedef struct _knob {
    uint8_t clkpin; 
    uint8_t datpin;
    uint8_t clklast;
    uint8_t val;
} knob;

knob knobs[num_knobs] = {
    { 2,3, 0, 0},    
    { 4,5, 0, 0},    
    { 6,7, 0, 0}, 
};

//-- knob data ---------------

void toggleLed() {
    digitalWrite( ledPin, digitalRead(ledPin)==HIGH ? LOW : HIGH );

}

static void knobs_init(void)
{
    for( int i=0; i<num_knobs; i++ ) {
        pinMode( knobs[i].clkpin, INPUT );
        pinMode( knobs[i].datpin, INPUT );
        // turn on internal pullup resistors so we don't need external ones
        digitalWrite( knobs[i].clkpin, HIGH);
        digitalWrite( knobs[i].datpin, HIGH);
    }
 
}

// this function must be called as quickly and as regularly as possible
static void knobs_poll(void)
{
    byte c,d;  // holder for readings
    for( byte i=0; i<num_knobs; i++ ) {
        knob k = knobs[i];           // get a knob
        c = digitalRead( k.clkpin ); // read its pins
        d = digitalRead( k.datpin );
        if( c != k.clklast  ) {      // look for clk line transition
            d = c^d;                   // xor gives us direction
            if( d ) k.val++;           // non-zero means clockwise rotation
            else k.val--;              // zero means counter-clockwise rotation
            k.clklast = c;             // save the clk pin's state
            knobs[i] = k;              // save our changes
        }
    }
}

// turn sawtooth into ramp
static byte knobs_fixval(byte b) {
    return (b<128) ? 2*b : 2*(255-b);
    //  return (b<85) ? 3*b : ((b<2*85) ? 3*(2*85-b) : 3*((3*85)-b));
}

void setup()
{
    Serial.begin(19200);

    //Use ledPin to flash when we get stuff
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, HIGH);

    knobs_init();

    BlinkM_begin();
  
    // set all BlinkMs to known state
    for( int i=0; i<num_blinkms; i++) {
        BlinkM_stopScript( blinkm_start_addr + i );
        BlinkM_fadeToRGB( blinkm_start_addr + i, 0,0,0); // fade to black
    }

    Serial.print("BlinkMCylon ready\n");
    delay(300);
}

// every loop poll knobs
// when t1 rolls over, update blinkm
// when t2 rolls over, move to next blinkm 
void loop()
{
    knobs_poll();

    t1++;             // counts loops
    if( t1==0 ) {     // t1 has rolled over, update LED

        // this is where sensor reading would go
        byte rr = knobs[red_knob].val;
        byte gr = knobs[grn_knob].val;
        byte br = knobs[blu_knob].val;

        byte r = knobs_fixval(rr);
        byte g = knobs_fixval(gr);
        byte b = knobs_fixval(br);

        //byte r = 255;
        //byte g = 11;
        //byte b = 11;

        // move to next LED
        t2--;
        if( t2==0 ) {   // t2 has rolled over, do cylon thing
            t2 = max_t2;
            toggleLed();

            byte blinkm_addr = blinkm_start_addr + curr_blinkm;

            BlinkM_setRGB( blinkm_addr, r,g,b );  // set to color
            BlinkM_setFadeSpeed( blinkm_addr, fadespeed);
            BlinkM_fadeToRGB( blinkm_addr, 0,0,0);   // fade to black

            // debug
            if( debug ) {
                Serial.print("r,g,b:");
                Serial.print(r,HEX);Serial.print(",");
                Serial.print(g,HEX);Serial.print(",");
                Serial.print(b,HEX);Serial.print(":");
                Serial.print("curr_blinkm:"); Serial.print(curr_blinkm,DEC);
                Serial.print(", addr:"); Serial.println(blinkm_addr,HEX);
            }

            // prepare to move to the next cylon eye element
            curr_blinkm = curr_blinkm + incdec;
            if( incdec == 1 && curr_blinkm == num_blinkms-1 )
                incdec = -1;
            else if( incdec == -1 && curr_blinkm == 0 )
                incdec = 1;
        }

    }
}


