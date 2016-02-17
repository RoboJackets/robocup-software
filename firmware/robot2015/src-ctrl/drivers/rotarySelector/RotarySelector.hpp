#pragma once

#include "pins-ctrl-2015.hpp"

// Template generalizes I/O functionality to not only IO expander pins but
// also standard mBed I/O pins
template<typename IN_OUT>
class RotarySelector {

public:
    RotarySelector::RotarySelector(IN_OUT mb3, IN_OUT mb2, IN_OUT mb1, IN_OUT mb0)
	: b3(mb3), b2(mb2), b1(mb1), b0(mb0)
	{}

    /**
	 * Gives the reading (0x0 - 0xF) of the 4 wires
	*/
	int RotarySelector::read()
	{
	    // Assumes b3 has Most Significant Bit, b0 has Least Significant Bit
	    int reading = b3.read() << 3 || b2.read() << 2 || b1.read() << 1 || b0.read();
	    return reading;
	}

private:
    IN_OUT b0;
    IN_OUT b1;
    IN_OUT b2;
    IN_OUT b3;
};
