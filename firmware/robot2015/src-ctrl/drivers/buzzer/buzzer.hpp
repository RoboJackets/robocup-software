#pragma once

#include <mbed.h>
#include <rtos.h>

#include "pins-ctrl-2015.hpp"

/**
 * @brief A buzzer for controlling a pieze element to play sounds using a DAC.
 */
class Buzzer : public AnalogOut {
public:
    Buzzer(PinName = RJ_SPEAKER) : AnalogOut(RJ_SPEAKER){};
    ~Buzzer(){};

    void play(float freq, int dur, float vol = 1.0);

private:
    friend void analogUpdate(void const* args);

    int j;
};
