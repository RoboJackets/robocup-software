#pragma once

#include <mbed.h>

#include "mcp23017.hpp"

#include "pins-ctrl-2015.hpp"


/*
 * Pin numbers that correspond to the MCP23017 datasheet:
 * http://ww1.microchip.com/downloads/en/DeviceDoc/21952b.pdf
 */
enum IOExpanderPin {
  IOExpanderPinA0 = 0,
  IOExpanderPinA1 = 1,
  IOExpanderPinA2 = 2,
  IOExpanderPinA3 = 3,
  IOExpanderPinA4 = 4,
  IOExpanderPinA5 = 5,
  IOExpanderPinA6 = 6,
  IOExpanderPinA7 = 7,
  IOExpanderPinB0 = 8,
  IOExpanderPinB1 = 9,
  IOExpanderPinB2 = 10,
  IOExpanderPinB3 = 11,
  IOExpanderPinB4 = 12,
  IOExpanderPinB5 = 13,
  IOExpanderPinB6 = 14,
  IOExpanderPinB7 = 15
};


/*
 * A digitalOut class meant to replicate basic functionality of the mBed digitalOut
 */
class IOExpanderDigitalOut
{
 private:
  IOExpanderPin m_pin;
  static bool isInit;

 public:
  // Default constructor will setup the hardware
  IOExpanderDigitalOut()
  {
    if (isInit == true)
      return;

    isInit = true;
  }

  // Other constructors for creating objects for pinouts
  IOExpanderDigitalOut(IOExpanderPin)
  {
    if (isInit == true)
      return;

    isInit = true;
  }

  /*
   * Pulls pin low if val = 0 and pulls pin high if val >= 1
   */
  void write(int const val)
  {
    MCP23017::write_bit(val, m_pin);

  }

  /*
   * Returns 0 if pin is low, 1 if pin is high
   */
  int read(void)
  {
    return MCP23017::read_bit(m_pin);
  }

  /*
   * Allows the equals operator to write to a pin
   */
  IOExpanderDigitalOut& operator= (int pin)
  {
    write(pin);
    return *this;
  }

  /*
   * Allows the equals operator to read the state of another IOExpander pin
   */
  IOExpanderDigitalOut& operator= (IOExpanderDigitalOut& rhs)
  {
    write(rhs.read());
    return *this;
  }

  /*
   * Allows the equals operator to read the state of another normal IO pin
   */
  IOExpanderDigitalOut& operator= (DigitalInOut& rhs)
  {
    write(rhs.read());
    return *this;
  }

  /*
   * Allows the pin to return its value like a simple integer variable
   */
  operator int(void)
  {
    return read();
  }
};
