#include <Servo.h>
#include <Wire.h>
#include <ArduinoNunchuk.h>


#define BAUDRATE 19200



// Constant servo actions
const int MS = 91; // angle that stops the motor
const int MCC = 140;
const int MC = 20;
// Important notes: 
//mounted in a certain position, forward is 20, backwards is 140
// But: For a car like this, to move a back wheel forward, you need to send it the backwards command
// And forward to go backwards!

// The leeway zone
const int leeway = 2;

// Constant joystick positions
const int analogCenterX = 133;
const int analogCenterY = 126;
const int analogLeft = 34;
const int analogRight = 227;
const int analogUp = 220;
const int analogDown = 27;

// The controller
ArduinoNunchuk nunchuk = ArduinoNunchuk();

// Array of servos
Servo servos[5];

// Array of pins the servos are attached to, index: {0=>Front Right, 1=>Back Right, 2=>Front Left, 3=>Back Left, 4=>plow}
int pins[] = {3, 10, 9, 11, 5};

void setup()
{
  // Begin serial communication
  Serial.begin(BAUDRATE);
  // Initialize the controller
  nunchuk.init();
  // Prep the servos
  setupServos();
}

// Constants for servos
#define FR servos[0] // Front right
#define FL servos[1] // Back right
#define BR servos[2] // Front left
#define BL servos[3] // Back left
#define PLOW servos[4] // Plow

void setupServos(){
  // Loop through the servos and attach a pin to it
  for(int i = 0 ; i < 5 ; i++){
    Serial.print("Attaching servo");
    Serial.print(' ');
    Serial.print(i, DEC);
    Serial.print(' ');
    Serial.print("to pin");
    Serial.print(' ');
    Serial.println(pins[i], DEC);
    servos[i].attach(pins[i]);
  } 
}

/**
Stops all the servos
**/
void stopServos(){
  FR.write(MS);
  BR.write(MS);
  FL.write(MS);
  BL.write(MS);
} 

void stopPlow(){
 PLOW.write(MS); 
}

void loop()
{

  // Update the controller
  nunchuk.update();
  
  // Begin Plow
  
  if(nunchuk.cButton == 1){
    // C Button pressed!
     PLOW.write(MCC);
  } else if (nunchuk.zButton == 1){
   // ZButton is pressed!
    PLOW.write(MC); 
  } else {
   stopPlow(); 
  }
  
  
  // End plow
  
  // If the joystick is in a position where it will not be affected, stop them all
  if((nunchuk.analogX < nunchuk.analogX + leeway || nunchuk.analogY < nunchuk.analogY + leeway)&&(nunchuk.analogX > nunchuk.analogX - leeway || nunchuk.analogY > nunchuk.analogY - leeway)){
   stopServos(); 
  }
  
  
  // If it's in the right side of the joystick (Q 1 or 4)
  if(nunchuk.analogX > analogCenterX + leeway){
    
    // If it's in Q1 (top right most quadrant of the joystick)
    if(nunchuk.analogY > analogCenterY + leeway){
      FL.write(MCC);
      BL.write(MCC);
      
      int xG = map(nunchuk.analogX, analogCenterX + leeway, analogRight, MS + 1, MC);
      int yG = map(nunchuk.analogY, analogCenterY + leeway, analogUp, MS + 1, MC);
      
      int xyAvg = (xG + yG)/2;
      /*
      Serial.print(yG, DEC);
      Serial.print(' ');
      Serial.print(xG, DEC);
      
      Serial.print(' ');
      Serial.println(yG, DEC);
      */
      
      FR.write(xyAvg);
      BR.write(xyAvg);

      
    } else /* If it's in Q4 (bottom right most quadrant of the joystick)*/
    if(nunchuk.analogY < analogCenterY - leeway){
      FL.write(MC);
      BL.write(MC);
      
      int xG = map(nunchuk.analogX, analogCenterX + leeway, analogRight, MS+1, MCC);
      int yG = map(nunchuk.analogY, analogCenterY - leeway, analogDown, MS+1, MCC);
      
      int xyAvg = (xG + yG)/2;
      
      FR.write(xyAvg);
      BR.write(xyAvg);
      /**
      FR.write(MCC);
      BR.write(MCC);
      
      int xG = map(nunchuk.analogX, analogCenterX + leeway, analogLeft, MS + 1, MC);
      int yG = map(nunchuk.analogY, analogCenterY - leeway, analogDown, MS - 1, MC);
      
      int xyAvg = (xG + yG)/2;
      
      FL.write(xyAvg); 
      BL.write(xyAvg);
      */
    }
    
  } else /* If it's in the left side of the joystick (Q 2 or 3) */ 
  if(nunchuk.analogX < analogCenterX + leeway){
    
    // If it's in Q2 (top left most quadrant of the joystick)
    if(nunchuk.analogY > analogCenterY + leeway){
      FR.write(MC);
      BR.write(MC);
      
      int xG = map(nunchuk.analogX, analogCenterX - leeway, analogLeft, MS + 1, MCC);
      int yG = map(nunchuk.analogY, analogCenterX + leeway, analogUp, MS + 1, MCC);
      
      
      int xyAvg = (xG + yG)/2;
      
      FL.write(xyAvg);
      BL.write(xyAvg);
    } else /*If it's in Q3 (bottom left most quadrant of the joystick)*/
    if(nunchuk.analogY < analogCenterY - leeway){
      FR.write(MCC);
      BR.write(MCC);
      
      int xG = map(nunchuk.analogX, analogCenterX - leeway, analogLeft, MS+1, MC);
      int yG = map(nunchuk.analogY, analogCenterY - leeway, analogDown, MS+1, MC);
      
      int xyAvg = (xG + yG)/2;
      
      FL.write(xyAvg);
      BL.write(xyAvg);
    }
    
  }
}

