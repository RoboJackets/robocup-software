#pragma once

#include "pins-ctrl-2015.hpp"

// Template generalizes I/O functionality to not only IO expander pins but
// also standard mBed I/O pins
template<typename IN_OUT>
class RotarySelector {

public:
    RotarySelector(IN_OUT mb3, IN_OUT mb2, IN_OUT mb1, IN_OUT mb0);

    int read();

private:
    IN_OUT b0;
    IN_OUT b1;
    IN_OUT b2;
    IN_OUT b3;
};
