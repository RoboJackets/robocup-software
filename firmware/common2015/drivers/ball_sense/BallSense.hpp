#pragma once

#include <mbed.h>
#include <rtos.h>
#include "RtosTimerHelper.hpp"

/// Determines if the emitter to reciever beam is broken while accounting for
///  ambiant light.
class BallSense {
public:
    BallSense(DigitalOut emitter, AnalogIn detector,
              DigitalOut ballSenseStatusLED);

    /// Returns true if beam is broken (Ball is contained)
    bool have_ball();

    /// Begin updating the sensor every @updateInterval ms
    void start(uint32_t updateInterval) { _updateTimer.start(updateInterval); }

    /// Stop updates
    void stop() { _updateTimer.stop(); }

private:
    // Call periodically when the update timer fires
    void update_ball_sensor();

    DigitalOut emitter_pin;
    AnalogIn detector_pin;

    DigitalOut ball_status_pin;

    // Holds the light sensed when
    //  emitter is lit (sense_light) and
    //  dark (sense_dark)
    uint16_t sense_dark = 0;

    // Emitter state (Light or dark)
    bool emitter_on = false;

    // If the light reading and dark reading are closer than this value, the
    // beam is considered broken
    const int sense_threshold = 500;

    // Number of consecative "broken" senses
    //  before we are confident the beam is broken
    const unsigned int consec_num = 2;

    // Consecative "broken" senses counter
    unsigned int consec_ctr = 0;

    RtosTimerHelper _updateTimer;
};
