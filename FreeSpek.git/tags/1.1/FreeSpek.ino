/**
 * Sketch for outputting a Spektrum compatible serial signal (125k baud) based on data read from a 6 channel receiver (Futaba). 
 * This code is provided as it is, no responability can be assumed for the consequences. Use at your own risk.
 *
 * Tested on an Arduino Pro Mini (3.3V / 8MHz). 
 * Pins:
 *   Digital: 
 *     2-7: Receiver Signal 
 *      10: red led
 *      12: green led
 *   Serial:
 *     TX0: Serial output to Spektrum TX Module
 *   Analog:
 *      A0: Used for battery voltage measurement, over a voltage divisor
 *
 * Based on the awsome work of folks at rc-heli-fan.org, rc-groups.com and rc-heli.de
 *
 * Used libraries:
 *  Timer: http://playground.arduino.cc/Code/Timer
 *  PinChangeInt: http://code.google.com/p/arduino-pinchangeint/
 */
 
//#define FREE_SPEK_DEBUG

#ifdef FREE_SPEK_DEBUG
//#define FREE_SPEK_DEBUG_SAMPLING 1000
#endif

#ifndef FREE_SPEK_DEBUG
#define TX_ACTIVE
#endif

#define RX_ACTIVE

#define DE_JITTER;

#define DSMX
//#define FRANCE


#ifdef RX_ACTIVE
/* we use only pins 2-7 at the moment, which is handled by port D alone :) */
#define NO_PORTB_PINCHANGES
#define NO_PORTC_PINCHANGES
#define DISABLE_PCINT_MULTI_SERVICE
#include <PinChangeInt.h>
#endif

#include <Event.h>
#include <Timer.h>

const byte RED = 10;
const byte GREEN = 12;
const byte BATTERY = A0;


const byte CHANNEL_TO_PINS_MAPPING[] = { 2 /* AIL */, 3 /* ELE */, 4 /* THR */, 5 /* RUD */, 6 /* GYRO - RX_STATUS */, 7 /* PIT */};
const byte CHANNEL_COUNT = sizeof(CHANNEL_TO_PINS_MAPPING);

const byte THROTTLE_CHANNEL = 3; /* futaba channel 3 */
const byte RX_STATUS_CHANNEL = 5; /* futaba channel 5 */

const short RX_STATUS_CHANNEL_BIND_VALUE = -600;
const short RX_STATUS_CHANNEL_RANGE_CHECK_VALUE = 600;


const byte DSM_CHANNELS[] = { 3, 1, 2, 4, 5, 6};
const byte DSM_CHANNELS_COUNT = sizeof(DSM_CHANNELS);

const byte DSM_HEADER_SIZE = 2;
const byte DSM_DATA_SIZE = 2 * DSM_CHANNELS_COUNT;

const word DSM_MIN_WORD_VALUE = 0;
const word DSM_MAX_WORD_VALUE = 1023;

const word DSM_SEND_INTERVAL = 19; 


#ifdef DE_JITTER
const byte DE_JITTER_MIN_PULSE_LEN_CHANGE = 16;
#endif


const short MIN_PULSE_LENGTH[CHANNEL_COUNT] = {  928,  912,  912,  912,  912,  928};
const short MED_PULSE_LENGTH[CHANNEL_COUNT] = { 1520, 1504, 1504, 1504, 1504, 1520};
const short MAX_PULSE_LENGTH[CHANNEL_COUNT] = { 2104, 2088, 2088, 2088, 2088, 2104};

const short MIN_CHANNEL_VALUE = -1400;
const short MAX_CHANNEL_VALUE = 1400;
const short MED_CHANNEL_VALUE = (MAX_CHANNEL_VALUE + MIN_CHANNEL_VALUE) / 2;


const float BATTERY_CELL_WARN_VOLTAGE = 3.7;
const float BATTERY_DIVISOR_MULTIPLIER = 4.045;
const word CHECK_BATTERY_INTERVAL = 60000; // every minute


const byte LED_UPDATE_INTERVAL = 100;


#ifdef FREE_SPEK_DEBUG
boolean debug = true;
#ifdef FREE_SPEK_DEBUG_SAMPLING
byte sampleCounter = 0;
#endif
#endif

word pulseLength[CHANNEL_COUNT];
word lastPulseLength[CHANNEL_COUNT];
unsigned long pulseBegin[CHANNEL_COUNT];

int redLedEvent;
int greenLedEvent;

Timer timer;

enum TX_STATE {
  disabled = -1,
  normal = 10,
  bind = 1,
  range_check = 5
};
#ifdef TX_ACTIVE
TX_STATE txState = normal;
#else
TX_STATE txState = disabled;
#endif

enum BATTERY_STATE {
  off = 5,
  ok = 20,
  low = 1
};
BATTERY_STATE batteryState = ok;

byte ledUpdate;
void updateLeds() {
  ledUpdate++;
  
  if (txState > 0 && ledUpdate % txState == 0) {
    digitalWrite(GREEN, 1 ^ digitalRead(GREEN));
  } else {
   digitalWrite(GREEN, LOW);
  }
  
  if (batteryState > 0 && ledUpdate % batteryState == 0) {
    digitalWrite(RED, 1 ^ digitalRead(RED));
  } else {
   digitalWrite(RED, LOW);
  }
}

float measureBattery() {
  const word sensorValue = analogRead(BATTERY);
  const float batteryValue = BATTERY_DIVISOR_MULTIPLIER * sensorValue * (3.3 / 1023.0);
  
  #ifdef FREE_SPEK_DEBUG
  Serial.print("Battery : ");
  Serial.print(batteryValue);
  Serial.print("V");
  Serial.println();
  #endif
  
  return batteryValue;
}

byte determineNumberOfBatteryCells() {
  return determineNumberOfBatteryCells(measureBattery());
}

byte determineNumberOfBatteryCells(float batteryValue) {
  return round(batteryValue / BATTERY_CELL_WARN_VOLTAGE);
}

void updateBatteryState() {
  const float batteryValue = measureBattery();
  const byte batteryNumberOfCells = determineNumberOfBatteryCells(batteryValue);
  batteryState = ok;
  if (batteryNumberOfCells == 0) {
    batteryState = off;
  } else if (batteryValue < batteryNumberOfCells * BATTERY_CELL_WARN_VOLTAGE) {
    batteryState = low;
    #ifdef FREE_SPEK_DEBUG
    Serial.println("Battery low!");
    #endif
  }
}

word mapChannelValueToPulse(short value, byte channelIndex) {
  value = constrain(value, MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE);
  if (value > MED_CHANNEL_VALUE) {
    return map(value, MED_CHANNEL_VALUE, MAX_CHANNEL_VALUE, MED_PULSE_LENGTH[channelIndex], MAX_PULSE_LENGTH[channelIndex]);
  } else {
    return map(value, MIN_CHANNEL_VALUE, MED_CHANNEL_VALUE, MIN_PULSE_LENGTH[channelIndex], MED_PULSE_LENGTH[channelIndex]);
  }
}

short mapPulseToChannelValue(word channelPulseLength, byte channelIndex) {
  channelPulseLength = constrain(channelPulseLength, MIN_PULSE_LENGTH[channelIndex], MAX_PULSE_LENGTH[channelIndex]);
  if (channelPulseLength > MED_PULSE_LENGTH[channelIndex]) {
    return map(channelPulseLength, MED_PULSE_LENGTH[channelIndex], MAX_PULSE_LENGTH[channelIndex], MED_CHANNEL_VALUE, MAX_CHANNEL_VALUE);
  } else {
    return map(channelPulseLength, MIN_PULSE_LENGTH[channelIndex], MED_PULSE_LENGTH[channelIndex], MIN_CHANNEL_VALUE, MED_CHANNEL_VALUE);
  }
}

word mapPulseToDSMChannelValue(word channelPulseLength, byte channelIndex) {
  channelPulseLength = constrain(channelPulseLength, MIN_PULSE_LENGTH[channelIndex], MAX_PULSE_LENGTH[channelIndex]);
  if (channelPulseLength > MED_PULSE_LENGTH[channelIndex]) {
    return map(channelPulseLength, MED_PULSE_LENGTH[channelIndex], MAX_PULSE_LENGTH[channelIndex], DSM_MAX_WORD_VALUE/2, DSM_MAX_WORD_VALUE);
  } else {
    return map(channelPulseLength, MIN_PULSE_LENGTH[channelIndex], MED_PULSE_LENGTH[channelIndex], DSM_MIN_WORD_VALUE, DSM_MAX_WORD_VALUE/2);
  }
}

#ifdef FREE_SPEK_DEBUG
void printCurrentValues(byte dsmHeader[], byte dsmData[], word actualPulseLength[]) {
  Serial.print(dsmHeader[0], HEX);
  Serial.print(dsmHeader[1], HEX);
  Serial.print(" | ");

  byte counter;
  for (counter = 0; counter < DSM_CHANNELS_COUNT; counter++) {
    byte dataByte1 = dsmData[2 * counter];
    byte dataByte2 = dsmData[2 * counter + 1];
    //word channelIndex = DSM_CHANNELS[counter] - 1;
    
    Serial.print(dataByte1, HEX);
    Serial.print(dataByte2, HEX);
    Serial.print(" ");
    //Serial.print("(K");
    //Serial.print((dataByte1 >> 2) + 1, DEC);
    //Serial.print("/");
    //Serial.print(DSM_CHANNELS[counter]);
    //Serial.print(": ");
    //Serial.print(((dataByte1 & ((1 << 2) - 1)) << 8) | dataByte2, DEC);
    //Serial.print("/");
    //Serial.print(mapPulseToChannelValue(actualPulseLength[channelIndex], channelIndex));
    //Serial.print("/");
    //Serial.print(actualPulseLength[channelIndex]);
    //Serial.print(")  ");
  }
  Serial.println();
}
#endif

void writeCurrentValues() {
  word actualPulseLength[CHANNEL_COUNT];

  memcpy(actualPulseLength, (const void*)pulseLength, sizeof(actualPulseLength));

  byte dsmHeader[DSM_HEADER_SIZE] = {0, 0};
  byte dsmData[DSM_DATA_SIZE];

  /**
  Header Bit1:
  bit 7 - 1 -> bind mode enable
  bit 6 - unknown, always 0
  bit 5 - 1 -> range test enable
  bit 4 - 1 -> normal mode, 0 -> france mode
  bit 3 - 1 -> DSMX enable
  bit 2 - 0 -> 0 - unknown, always 0
  
  Header Bit2:
  bit 7 - 0 -> 0 - unknown, always 0
  
  **/    

  #ifdef DSMX
  // set DSMX header flag (bit 3)
  dsmHeader[0] |= (1 << 3);
  #endif

  #ifndef FRANCE
  // set normal send header (bit 4)
  dsmHeader[0] |= (1 << 4);
  #endif

  byte counter;
  for (counter = 0; counter < DSM_CHANNELS_COUNT; counter++) {
    byte channelDataByte1 = 0;
    byte channelDataByte2 = 0;

    const byte channel = DSM_CHANNELS[counter];
    const byte channelIndex = channel - 1;
    
    word channelPulseLength = actualPulseLength[channelIndex];
    
    // check if pulse is valid
    if (channelPulseLength < MIN_PULSE_LENGTH[channelIndex] || channelPulseLength > MAX_PULSE_LENGTH[channelIndex]) {
      #ifdef FREE_SPEK_DEBUG
      if (debug) {
        Serial.print("Invalid channel pulse length: ");
        Serial.print(channelPulseLength);
        Serial.print(" @ channel: ");
        Serial.print(channelIndex + 1);
        Serial.println();
      }
      #endif
      
      channelPulseLength = lastPulseLength[channelIndex];
    } else {      
      // valid pulse
      #ifdef DE_JITTER
      const word lastChannelPulseLength = lastPulseLength[channelIndex];
      if (abs(channelPulseLength - lastChannelPulseLength) < DE_JITTER_MIN_PULSE_LEN_CHANGE) {
        channelPulseLength = lastPulseLength[channelIndex];
      }
      #endif
      lastPulseLength[channelIndex] = channelPulseLength;  
    }
    
    const word channelValue = mapPulseToDSMChannelValue(channelPulseLength, channelIndex);

    if (channel == RX_STATUS_CHANNEL) {
      // set header based on channel value
      const short value = mapPulseToChannelValue(channelPulseLength, channelIndex);
      if (value < min(RX_STATUS_CHANNEL_BIND_VALUE, RX_STATUS_CHANNEL_RANGE_CHECK_VALUE)) {
        // set bind (bit 7)
        dsmHeader[0] |= (1 << 7);
        txState = bind;
      } else if (value > max(RX_STATUS_CHANNEL_BIND_VALUE, RX_STATUS_CHANNEL_RANGE_CHECK_VALUE)) {
        // set range check (bit 5)
        dsmHeader[0] |= (1 << 5);
        txState = range_check;
      } else if (txState == bind || txState == range_check) {
        txState = normal;
      }
    }

    /*
      Bit 1      Bit2
      00xx xxyy  yyyy yyyy
      |- first 2 bits = 0
        |- xxxx = 4 channel number bits (0 - 15) - 
             |- yyyyyyyyyy = 10 channel value bits (0 - 1023)      
     */
    channelDataByte1 |= (counter & ((1 << 4) - 1) ) << 2;
    channelDataByte1 |= (channelValue >> 8) & ((1 << 2) - 1);
    channelDataByte2 |= (channelValue & ((1 << 8) - 1));
    
    dsmData[2 * counter] = channelDataByte1;
    dsmData[2 * counter + 1] = channelDataByte2;
  }
  
  #ifdef FREE_SPEK_DEBUG
  if (debug) {
    printCurrentValues(dsmHeader, dsmData, actualPulseLength);
  }
  #endif

  #ifdef TX_ACTIVE
  Serial.write(dsmHeader, DSM_HEADER_SIZE);
  Serial.write(dsmData, DSM_DATA_SIZE);
  #endif
}

#ifdef RX_ACTIVE
void pinChanged() {
  /* TODO: find a way for reverse mapping of pin to channel */
  const byte channelIndex =  PCintPort::arduinoPin - 2;
  if (PCintPort::pinState) {
    // pin goes up (pulse begins) so need to save current time
    pulseBegin[channelIndex] = micros();
  }  else {
    // pin goes down, pulse ends so update the pulse length
    pulseLength[channelIndex] = micros() - pulseBegin[channelIndex];
  }
}
#endif

void setup() {   
  delay(50);
  
  #ifdef TX_ACTIVE
  Serial.begin(125000);
  #else
  Serial.begin(115200);
  #endif
  
  delay(20);

  pinMode(RED, OUTPUT);
  digitalWrite(RED, LOW);
  
  pinMode(GREEN, OUTPUT);
  digitalWrite(GREEN, LOW);

  pinMode(BATTERY, INPUT);
  
  // set initial values:
  memset(pulseLength,0,sizeof(pulseLength));
  memset(lastPulseLength,0,sizeof(lastPulseLength));
  memset(pulseBegin,0,sizeof(pulseBegin));

  delay(20);

  byte counter;
  for (counter = 0; counter < CHANNEL_COUNT; counter++) {
    // init pins
    const byte pin = CHANNEL_TO_PINS_MAPPING[counter];
    pinMode(pin, INPUT);  
    digitalWrite(pin, HIGH);
    
    #ifdef RX_ACTIVE
    PCintPort::attachInterrupt(pin, &pinChanged, CHANGE);
    #endif
    
    // set "fail-safe" values
    pulseLength[counter] = MED_PULSE_LENGTH[counter];
    if (counter + 1 == THROTTLE_CHANNEL) {
      pulseLength[counter] = MIN_PULSE_LENGTH[counter];
    }
  }

  delay(20);

  #ifdef FREE_SPEK_DEBUG
  Serial.println("Fail safe (initial) values:");
  writeCurrentValues();    
  Serial.println();
  #endif
  
  delay(20);
  
  const byte batteryNumberOfCells = determineNumberOfBatteryCells();
  for (counter = 0; counter < batteryNumberOfCells; counter++) {
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, HIGH);
    delay(200);
    digitalWrite(RED, LOW);
    digitalWrite(GREEN, LOW);
    delay(200);
  }
  updateBatteryState();
  
  delay(20);
   
  timer.every(DSM_SEND_INTERVAL, &writeCurrentValues);
  timer.every(CHECK_BATTERY_INTERVAL, &updateBatteryState);
  timer.every(LED_UPDATE_INTERVAL, &updateLeds);
  
  delay(20);
  
  // flash LEDs in init sequence
  for (counter = 0; counter < 10; counter++) {
    boolean ledState = counter % 2 == 0;
    digitalWrite(RED, ledState ? HIGH : LOW);
    digitalWrite(GREEN, !ledState ? HIGH : LOW);
    delay(200);
  }
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
}

void loop() {
  #ifdef FREE_SPEK_DEBUG
  #ifdef FREE_SPEK_DEBUG_SAMPLING
  debug = (sampleCounter % FREE_SPEK_DEBUG_SAMPLING) == 0;
  sampleCounter = (sampleCounter + 1) % FREE_SPEK_DEBUG_SAMPLING;
  #endif
  #endif
  
  timer.update();
}
