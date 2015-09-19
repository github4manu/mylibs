/**
 * WiicooZ v1.2.0 20120405 by Randy Simons
 *
 * Use a Wii Nunchuck to control a PicooZ RC helicopter, like the Sidewinder.
 * 
 * Details:
 * http://randysimons.nl/pagina_142_NL.xhtml
 * http://www.silverlit-flyingclub.com/product_picooz.htm
 * http://www.rcgroups.com/forums/showatt.php?attachmentid=1252169
 * http://todbot.com/blog/2008/02/18/wiichuck-wii-nunchuck-adapter-available/
 *
 * Usage:
 * Connect nunchuck to arduino, hook up a IR-led to pin 9.
 * alter CHANNEL for your helicopter, power up arduino.
 * Press Z (and keep pressed) to enable transmission and fly away!
 *
 * Rudder: joystick left/right
 * Throttle: nunchuck up/down
 * Trim: C-button + joystick left/right
 **/

#define LED 13 //Indicator LED. Lights when IR-led is transmitting.
#define CHANNEL 1 //0 = channel A, 1 = channel B, 2 = channel C.
#define DELAY 70 //Send data to heli every 70ms (or so..)

//Limits for nunchuck values. You might have to adjust this for your own nunchuck.

#define THROTTLE_MAX 90 //Value of nunchuck accel y for maximum throttle
#define THROTTLE_MIN 170 //Value of nunchuck accel y for no throttle

#define RUDDER_LEFT 28 //Value of nunchuck joystick X for left
#define RUDDER_RIGHT 233 //Value of nunchuck joystick X for right

#define RUDDER_RANGE 100 //Should be <RUDDER_RIGHT-RUDDER_LEFT

#include <Wire.h>
#include "nunchuck_funcs.h"

bool bEnabled;
int iJoyCenter, iErrorThrottle, iErrorRudder;

void setup() {
  pinMode(LED, OUTPUT); // Indicator LED.
  pinMode(9,OUTPUT); //PWM pin. Connect IR led on this pin.

  setup_timer1();
  Serial.begin(115200);

  nunchuck_setpowerpins();
  nunchuck_init();

  //Disable RC at start
  bEnabled=false; 

  iErrorThrottle=iErrorRudder=0;

  //Get center value for joystick
  //First value is bogus...  
  nunchuck_get_data();
  delay(200);
  nunchuck_get_data();
  iJoyCenter=nunchuck_joyx();
}

void loop() {
  int nunchuckX, nunchuckY, throttle, rudder, trim;

  nunchuck_get_data();

  nunchuckY =  constrain(nunchuck_accely(),min(THROTTLE_MIN,THROTTLE_MAX),max(THROTTLE_MIN,THROTTLE_MAX)) ; //min=170, max=90
  nunchuckX =  constrain(nunchuck_joyx(),min(RUDDER_LEFT,RUDDER_RIGHT),max(RUDDER_LEFT,RUDDER_RIGHT)); //left=28, right=233, center 130

  //Correct values for error diffusion
  throttle=nunchuckY+iErrorThrottle;  
  rudder=nunchuckX+iErrorRudder;

  //Don't like the error diffusion? use this:
  //throttle=nunchuckY;  
  //rudder=nunchuckX;

  //Scale down values. You might need to adjust this for your own nunchuck.
  throttle=fmap(throttle,THROTTLE_MIN,THROTTLE_MAX,0,14);
  rudder=fmap(rudder,iJoyCenter-RUDDER_RANGE,iJoyCenter+RUDDER_RANGE,-4,4); //Note: the +4 is for symetry only; it will be truncated.

  //Calculate the error, for error diffusion
  //use a error die-out-factor of 0.8
  iErrorThrottle=0.8*(iErrorThrottle+(nunchuckY-map(throttle,0,14,THROTTLE_MIN,THROTTLE_MAX)));
  iErrorRudder=0.8*(iErrorRudder+(nunchuckX-map(rudder,-4,4,RUDDER_LEFT,RUDDER_RIGHT)));

  //Limit range - actually only needed for rudder
  throttle=constrain(throttle,0,14);
  rudder=constrain(rudder,-4,3);


  //trim. Press c-button + joystick right to trim right, c-button + left to trim... left!
  if (nunchuck_cbutton()) {

    if (rudder<0)
      trim=-1;
    else if (rudder>0) {
      trim=1;
    }

    rudder=0;
  } 
  else {
    trim=0;
  }

  if (nunchuck_zbutton()) {  
    sendDataFrame(CHANNEL, throttle, rudder, trim);
    Serial.print("Throttle enabled; ");  
  } 
  else {
    sendDataFrame(CHANNEL, 0, 0, trim);
    Serial.print("Throttle disabled; ");    
  }

  Serial.print("Throttle: "); 
  Serial.print(throttle,DEC);  
  Serial.print(", Rudder: "); 
  Serial.print(rudder,DEC);
  Serial.print(", Trim: "); 
  Serial.println(trim,DEC);


  delay(DELAY); //The real remote uses different intervals, based on the channel
}

/**
 * Similar to the built-in map, but uses floats al the way, and rounds instead of truncates.
 */
int fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return lround((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void setup_timer1() {

  //Use timer1 for frequency-correct PWM, with variable duty cycle.

  noInterrupts();   
  ICR1=420; //421 clockticks @ 16MHz = 38kHz carrier wave
  OCR1A=0.5*420; //50% duty cycle. Smaller duty cycles allow for brighter flashes.
  interrupts();

  TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (1<<WGM11) | (0<<WGM10); //Disconnect pin for now
  TCCR1B=(1<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
}

void pulse(int on, int off) {
  int i;

  digitalWrite(LED,HIGH);
  TCCR1A|=_BV(COM1A1); // connect pulse clock
  delayMicroseconds(on); 

  digitalWrite(LED,LOW);  
  TCCR1A&=~_BV(COM1A1); // disconnect pulse clock
  delayMicroseconds(off);  
}

int sendBit(int bit) {
  if (bit) {
    pulse(1100,1100);
  } 
  else {    
    pulse(550,550);
  }
}

int getCode(int channel, int throttle, int trim, int rudder) {
  int code=0; //Format: CCTT TTBB BBRR R000

    channel&=3;
  throttle&=15;
  trim&=15;
  rudder&=7;

  code=channel<<14 | throttle<<10 | trim<<6 | rudder<<3;

  return code;
}

/**
 * Calculate checksum
 */
int getChecksum(int code) {
  int checksum=0;

  for (int i=2; i<=14; i+=2) {
    checksum+=(code>>i);
  }

  checksum=(-checksum)&3;

  return checksum;
}



/**
 * Sends a complete dataframe, with start and end delimiters, checksum.
 *
 * channel: a=0, b=1, c=2
 * throttle: 0-14. boost: 15 (?)
 * rudder: -4 - +3 (?)
 * trim: right: 1, left: -1
 */
void sendDataFrame(int channel, int throttle, int rudder, int trim) {
  int code=getCode(channel, throttle, trim, rudder);
  int checksum=getChecksum(code);

  //start
  pulse(1900,600);
  pulse(700,600);
  pulse(700,600);
  pulse(1200,600);
  pulse(1200,600);

  //code
  for (int i=15;i>2;i--) {
    sendBit(code&(1<<i));
  }

  //checksum
  sendBit(checksum&1);
  sendBit(checksum&2);

  //stop
  pulse(700,600);
}
