#pragma once

#include <mbed.h>

#include "pins-ctrl-2015.hpp"
#include "mcp23017.hpp"

/*
 * A digitalInOut class meant to replicate basic functionality of the
 * mBed digitalOut and digitalIn
 */
class IOExpanderDigitalInOut {
private:
    IOExpanderPin m_pin;

public:
    // Default constructor will setup the hardware
    IOExpanderDigitalInOut(){};

    // Other constructors for creating objects for pinouts
    IOExpanderDigitalInOut(IOExpanderPin pin, bool state = false) {
        m_pin = pin;

        if (state != read()) write(state);
    }

    /*
     * Pulls pin low if val = 0 and pulls pin high if val >= 1
     */
    void write(int const val) { MCP23017::write_bit(val, m_pin); }

    /*
     * Returns 0 if pin is low, 1 if pin is high
     */
    int read() { return MCP23017::read_bit(m_pin); }

    /*
     * Allows the equals operator to write to a pin
     */
    IOExpanderDigitalInOut& operator=(int pin) {
        write(pin);
        return *this;
    }

    /*
     * Allows the equals operator to read the state of another IOExpander pin
     */
    IOExpanderDigitalInOut& operator=(IOExpanderDigitalInOut& rhs) {
        write(rhs.read());
        return *this;
    }

    /*
     * Allows the equals operator to read the state of another normal IO pin
     */
    IOExpanderDigitalInOut& operator=(DigitalInOut& rhs) {
        write(rhs.read());
        return *this;
    }

    /*
     * Allows the pin to return its value like a simple integer variable
     */
    operator int() { return read(); }
};
