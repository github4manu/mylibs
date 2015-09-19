// Arduino based interface between a (wired) standard Wii Nunchuck and another device
// In my case this is a twin wheeled electric skateboard. The variable voltage outputs from Arduino go to the main microcontroller which is controlling
// and balancing the board.
// Uses Duemilanove ATMega328  Arduino 17 with wired Wii Nunchuck.

// 5V output on Arduino to Nunchuck (works better than using 3.3V line)
// GND
// SDA to analog pin 4
// SCK to analog pin 5
// the middle upper and middle lower contacts of nunchuck plug are not used

//Derived from code developed by Chad Phillips, see:  http://windmeadow.com/node/42
//   For info on wiring up the chuck to the Arduino, download Bionic Arduino Class 4 from TodBot blog:
//   http://todbot.com/blog/2007/11/24/bionic-arduino-class-notes-3-4/

// To avoid hacking off plug you can buy a nunchuck extender cable (plug one end socket the other) and 
// cut that in half to get the socket on a cable, with free wires on the other end to attach to Arduino

// Some non-nintendo Nunchucks have shorter cables by the way than the real thing.

//Outputs:
//The outputs from the Arduino are arranged as follows;
//Digital Pin 9: 
//outputs +5V when either of the end buttons are pressed. This can be used as a go/stop control signal. 
//If controlling a vehicle / robot or similar, then letting go of the Nunchuck will create a 0V at this output and so can be used to kill power to motors and so on.

//Digital Pin 10: 
//PWM (pulse width modulated) output on pin 10 gives 2.5V most of time. 
//NOTE: This is not a smooth 2.5V but a series of pulses at 500Hz where half the time there is 0V and half the time there is +5V. If you smooth them out using a resistor and a capacitor (called an RC filter) you can use the resulting (now smoothed) voltage as a control input signal for some other device (another arduino or whatever).
//Output is 2.5V when joystick in neutral position. 
//It Climbs to +5V or drops to 0V when joystick moved forwards or backwards. 

//Digital Pin 11: 
//PWM (pulse width modulated) output on pin 11 gives 2.5V most of time (in similar way to Pin 11 above) when joystick in neutral position. 
//If z button is pressed, then joystick left/right will make it go down to 0V or up to 5V. 

//BUT: If both z and c buttons pressed (or just c button alone) then this Pin 11 output becomes controlled by Left/Right TILT of the nunchuck (via embedded accelerometer) and the L/R joystick component is disabled.


//Step 1: View of Nunchuck buttons
//
//Here are the “c” and “z” buttons on front of the Nunchuck.
//
//Step 2: Arduino board inputs and outputs
//
//Outputs:
//The outputs from the Arduino are arranged as follows;
//Digital Pin 9: outputs +5V when either of the end buttons are pressed. This can be used as a go/stop control signal. If controlling a vehicle / robot or similar, then letting go of the Nunchuck will create a 0V at this output and so can be used to kill power to motors and so on.
//Digital Pin 10: PWM (pulse width modulated) output on pin 10 gives 2.5V most of time. NOTE: This is not a smooth 2.5V but a series of pulses at 500Hz where half the time there is 0V and half the time there is +5V. If you smooth them out using a resistor and a capacitor (called an RC filter) you can use the resulting (now smoothed) voltage as a control input signal for some other device (another arduino or whatever).
//Output is 2.5V when joystick in neutral position.
//It climbs to +5V or drops to 0V when joystick moved forwards or backwards.
//Digital Pin 11: PWM (pulse width modulated) output on pin 11 gives 2.5V most of time (in similar way to Pin 11 above) when joystick in neutral position. If z button is pressed, then joystick left/right will make it go down to 0V or up to 5V.
//BUT: If both z and c buttons pressed (or just c button alone) then this Pin 11 output becomes controlled by Left/Right TILT of the nunchuck (via embedded accelerometer) and the L/R joystick component is disabled.
//So: The output voltage (smoothed with an R/C filter) can be used as a steering input to some other device. If pressing z button, they you steer with joystick, if c button (or both) is being pressed, then you steer by tilting Nunchuck left or right.
//For more detail: Wii Nunchuck as general purpose controller via Arduino board




#include <Wire.h>
#include <string.h>

#undef int
#include <stdio.h>

uint8_t outbuf[6];		// array to store arduino output
int cnt = 0;
int ledPin = 13;
int i; //declare i as an integer
int j; //declare j as an integer

#define LEDx 11   //pin 11 varies according to x accel  pin10 is a pwm
#define LEDy 10   //defines pin 10 as varying according to y accel pin 9 is a pwm
#define ledc 8   //defines pin 8 as output for c button
#define ledz 9   //defines pin 9 as output for z button


void
setup ()
{
  //Include this if you want to watch the data streaming from chuck in the serial viewer window on your PC
  //Serial.begin (19200);  
  //Serial.print ("Finished setup\n");
  pinMode(LEDx, OUTPUT); //tell arduino it is an output
  pinMode(LEDy, OUTPUT); //tell arduino it is an output
  pinMode(ledc, OUTPUT); //tell arduino it is an output
  pinMode(ledz, OUTPUT); //tell arduino it is an output
  
  Wire.begin ();		// join i2c bus with address 0x52
  nunchuck_init (); // send the initilization handshake
}

void
nunchuck_init ()
{
  Wire.beginTransmission (0x52);	// transmit to device 0x52
  Wire.send (0x40);		// sends memory address
  Wire.send (0x00);		// sends sent a zero.  
  Wire.endTransmission ();	// stop transmitting
}

void
send_zero ()
{
  Wire.beginTransmission (0x52);	// transmit to device 0x52
  Wire.send (0x00);		// sends one byte
  Wire.endTransmission ();	// stop transmitting
}

void
loop ()
{
  Wire.requestFrom (0x52, 6);	// request data from nunchuck
  while (Wire.available ())
    {
      outbuf[cnt] = nunchuk_decode_byte (Wire.receive ());	// receive byte as an integer
      digitalWrite (ledPin, HIGH);	// sets the LED on
      cnt++;
    }

  // If we recieved the 6 bytes, then go print them
  if (cnt >= 5)
    {
      print ();
    }

  cnt = 0;
  send_zero (); // send the request for next bytes
  delay (100);
}

// Print the input data we have recieved
// accel data is 10 bits long
// so we read 8 bits, then we have to add
// on the last 2 bits.  That is why I
// multiply them by 2 * 2
void
print ()
{
  int joy_x_axis = outbuf[0];
  int joy_y_axis = outbuf[1];
  int accel_x_axis = outbuf[2] * 2 * 2; 
  int accel_y_axis = outbuf[3] * 2 * 2;
  int accel_z_axis = outbuf[4] * 2 * 2;

  int z_button = 0;
  int c_button = 0;

 // byte outbuf[5] contains bits for z and c buttons
 // it also contains the least significant bits for the accelerometer data
 // so we have to check each bit of byte outbuf[5]
  if ((outbuf[5] >> 0) & 1)
    {
      z_button = 1;
    }
  if ((outbuf[5] >> 1) & 1)
    {
      c_button = 1;
    }

  if ((outbuf[5] >> 2) & 1)
    {
      accel_x_axis += 2;
    }
  if ((outbuf[5] >> 3) & 1)
    {
      accel_x_axis += 1;
    }

  if ((outbuf[5] >> 4) & 1)
    {
      accel_y_axis += 2;
    }
  if ((outbuf[5] >> 5) & 1)
    {
      accel_y_axis += 1;
    }

  if ((outbuf[5] >> 6) & 1)
    {
      accel_z_axis += 2;
    }
  if ((outbuf[5] >> 7) & 1)
    {
      accel_z_axis += 1;
    }

  /*  Include these commands if you want to watch the data values streaming from chuck in the serial viewer window on your PC
  Serial.print (joy_x_axis, DEC); //values seen: 37left 134mid  234right
  Serial.print ("\t");

  Serial.print (joy_y_axis, DEC); //values seen: 218forward  124mid  23back
  Serial.print ("\t");

  Serial.print (accel_x_axis, DEC); //values: 312tiltedleft  512mid  745tiltedright
  Serial.print ("\t");

  Serial.print (accel_y_axis, DEC); //values: tippedforward754  mid576   tippedback361
  Serial.print ("\t");

  Serial.print (accel_z_axis, DEC);  // values: hovers around 530 - 743
  Serial.print ("\t");

  Serial.print (z_button, DEC); //NB: 0 when pressed, 1 when not (i.e. opposite of what you might expect)
  Serial.print ("\t");

  Serial.print (c_button, DEC); //NB: 0 when pressed, 1 when not (i.e. opposite of what you might expect)
  Serial.print ("\t");

  Serial.print ("\r\n");
  */



j = joy_y_axis;
analogWrite(LEDy, j);  //sets PWM voltage output on pin 10 according to forward/back of joystick


// "led" terms are there because this software was originally written to make a set of LED's go dim or bright according to Nunchuck controls.
// If you put an LED onto Pins 9, 10, and 11 (with a resistor) you can use them to test this software is working correctly. 

if (z_button < 1 && c_button == 1) {  //if pressing z button only, steer on left/right joystick
                  digitalWrite (ledz, HIGH);	// sets the output high on pin 9 (i.e. +5V)
                  i = joy_x_axis - 6; //mid point is 134 this corrects it to 128
                  analogWrite(LEDx, i);  //sets PWM voltage output on pin 11
                                  } 
                                  
if (z_button < 1 && c_button < 1) {  //if pressing both buttons, steer using accelerometer
                  digitalWrite (ledz, HIGH);	// sets the output high on pin 9 (i.e. +5V)
                  i= (int) (accel_x_axis - 512) / 2 + 128;   //convert from range 312 to 512mid to 745 to a 0-255 scale
                  if (i>255) i=255;
                  if (i<0) i=0;
                  analogWrite(LEDx, i);  //sets PWM voltage output on pin 10
                                  } 


                                     
if (z_button == 1 && c_button < 1) {  //if c button only pressed then steer on accelerometer
                  digitalWrite (ledz, HIGH);	// sets the output high on pin 9 (i.e. +5V)
                  i= (int) (accel_x_axis - 512) / 2 + 128;   //convert from range 312 to 512mid to 745 to a 0-255 scale
                  if (i>255) i=255;
                  if (i<0) i=0;
                  analogWrite(LEDx, i);  //sets PWM voltage output on pin 11
                                    }
                    
if (z_button == 1 && c_button == 1) {   //if no button pressed then dead ahead plus control signal to Pin9 OFF i.e. kill motor if Pin9 being used as a "kill" switch
                  digitalWrite (ledz, LOW);	// sets the output low on pin 9, (i.e. 0V as neither button being pressed) 
                  analogWrite(LEDx, 128);  //sets PWM voltage output on pin 11 at half of maximum i.e. dead ahead if being used for steering a robot for example  
                  analogWrite(LEDy, 128);  //sets PWM voltage output on Pin 10 at half of maximum (i.e. 2.5V)          
                                     }
          
}

// Encode data to format that most wiimote drivers except
// only needed if you use one of the regular wiimote drivers
char
nunchuk_decode_byte (char x)
{
  x = (x ^ 0x17) + 0x17;
  return x;
}


