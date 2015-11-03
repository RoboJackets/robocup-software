#pragma once

#include <mbed.h>
#include <rtos.h>

#include "pins-ctrl-2015.hpp"

void analogUpdate(void const* args);

class Buzzer : public AnalogOut {
public:
    Buzzer() : AnalogOut(RJ_SPEAKER) {};
    ~Buzzer() {};

    void play(float freq, int dur, float vol = 1.0);

    int getIndex();
    void setIndex(int newIndex);

private:
    friend void analogUpdate(void const* args);

    int j;
};
