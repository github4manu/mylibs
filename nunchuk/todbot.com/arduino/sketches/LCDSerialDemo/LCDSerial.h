/*
  LCDSerial.cpp - Sparkfun LCD Serial comm using simple software serial 
  2008, Tod E. Kurt, http://todbot.com/blog/

  Based on:
  SoftwareSerial (2006) by David A. Mellis and
  AFSoftSerial (2008) by ladyada

  Copyright (c) 2006 David A. Mellis.  All right reserved. - hacked by ladyada 
*/

#ifndef _LCDSERIAL_H_
#define _LCDSERIAL_H_

#include <stdint.h>

// stolen from HardwareSerial.h
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0

class LCDSerial
{
  private:
    long _baudRate;
    uint8_t _transmitPin;
    void printNumber(unsigned long, uint8_t);

  public:
    LCDSerial(uint8_t lcdPin);
    void begin(long speed);
    void clearScreen(void);
    void gotoLineOne(void);
    void gotoLineTwo(void);
    void backlightOn(void);
    void backlightOff(void);
    void print(char);
    void print(const char[]);
    void print(uint8_t);
    void print(int);
    void print(unsigned int);
    void print(long);
    void print(unsigned long);
    void print(long, int);
 };

#endif
