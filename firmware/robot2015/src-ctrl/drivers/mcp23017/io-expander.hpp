#pragma once

#include <mbed.h>

#include "pins-ctrl-2015.hpp"
#include "mcp23017.hpp"

/**
 * A DigitalInOut class meant to replicate basic functionality of the
 * mBed DigitalOut and DigitalIn on the io-expander.
 */
class IOExpanderDigitalInOut {
public:
    IOExpanderDigitalInOut(MCP23017* mcp, IOExpanderPin pin,
                           MCP23017::PinMode mode, bool state = false)
        : _pin(pin), _mcp23017(mcp) {
        pinMode(mode);
        if (state != read()) write(state);
    }

    void pinMode(MCP23017::PinMode mode) { _mcp23017->pinMode(_pin, mode); }

    /// Pulls pin low if val = 0 and pulls pin high if val >= 1
    void write(int val) { _mcp23017->writeBit(val, _pin); }

    /// Returns 0 if pin is low, 1 if pin is high
    int read() { return _mcp23017->readBit(_pin); }

    /// Allows the equals operator to write to a pin
    IOExpanderDigitalInOut& operator=(int value) {
        write(value);
        return *this;
    }

    /// Allows the pin to return its value like a simple integer variable
    operator int() { return read(); }

private:
    IOExpanderPin _pin;
    MCP23017* _mcp23017;
};
