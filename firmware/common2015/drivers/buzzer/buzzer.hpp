#pragma once

#include <mbed.h>
#include <rtos.h>

/**
 * @brief A buzzer for controlling a pieze element to play sounds using a DAC.
 */
class Buzzer : public AnalogOut {
public:
    Buzzer(PinName pin) : AnalogOut(pin){};
    ~Buzzer(){};

    void play(float freq, int dur, float vol = 1.0);

private:
    void analogUpdate();

    int j;
};
