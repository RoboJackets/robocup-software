#pragma once

/**
 * This file provides some helper classes for emulating an mbed's hardware.  It
 * is useful for unit-testing code on a computer when an mbed is not available
 * or convenient.
 */
namespace fake_mbed {

class DigitalIn {
public:
    DigitalIn(int value) : _value(value) {}

    int read() const { return _value; }
    operator int() { return read(); }

private:
    int _value;
};
}
