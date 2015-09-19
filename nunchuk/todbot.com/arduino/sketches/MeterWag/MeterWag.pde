/*
 * MeterWag 
 * --------
 * 
 * Wag an analog meter back n forth
 * 
 * On the meter from the red Radio Shack battery tester,
 * a 4.7k resistor in series with the meter (which has ~30ohm resistance)
 * gives full scale reading when analogWrite(255) is done
 * (tho my voltmeter is saying that's actually 3.75V, wtf?)
 *
 * 2008, Tod E. Kurt, http://todbot.com/
 */

#define meterPin 11   // select the pin for the meter

int delaytime = 15;
byte mode = 0;      // 0 == wag, 1 = go random
int loopcnt;

byte val = 0;       // variable to store the value coming from the sensor
byte valinc = 1;

void setup() {
  pinMode(meterPin, OUTPUT);  // declare the ledPin as an OUTPUT
  Serial.begin(19200);
}

void loop() {
  analogWrite(meterPin, val);    // move the meter
  loopcnt++;

  if( mode == 1 ) { 
    val = random(20,220);
    delay(800);
    
    if( loopcnt==10 ) { 
      mode = !mode;
      loopcnt=0;
    }
  }
  else { 
    val += valinc;
    if( val==220 ) valinc = -1;
    else if( val== 20 ) valinc = 1;
    
    if( loopcnt==760 ) { 
      mode = !mode;
      loopcnt=0;
    }
  } 
  
  Serial.print("val: ");
  Serial.println(val, DEC);
  delay(delaytime);
}

