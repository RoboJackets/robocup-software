#include "RotarySelector.hpp"


RotarySelector::RotarySelector(DigitalInOut arg_b3, DigitalInOut arg_b2,
    DigitalInOut arg_b1, DigitalInOut arg_b0)
{
    b3 = arg_b3;
    b2 = arg_b2;
    b1 = arg_b1;
    b0 = arg_b0;
}

/**
 * Gives the reading (0x0 - 0xF) of the 4 wires
 */
int RotarySelector::read()
{
    // Assumes b3 has Most Significant Bit, b0 has Least Significant Bit
    int reading = b3.read() << 3 || b2.read() << 2 || b1.read() << 1 || b0.read();
    return reading;
}
