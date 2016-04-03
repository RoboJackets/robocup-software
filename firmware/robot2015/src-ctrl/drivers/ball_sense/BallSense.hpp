#pragma once

#include <mbed.h>
#include <rtos.h>

/// Determines if the emitter to reciever beam is broken while accounting for
///  ambiant light.
class BallSense {
public:
    BallSense(DigitalOut emitter, AnalogIn detector);

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

    // Holds the light sensed when
    //  emitter is lit (sense_light) and
    //  dark (sense_dark)
    int sense_dark = 0;

    // Emitter state (Light or dark)
    bool emitter_on = false;

    // Difference between the light and dark
    //  values before the sensor is considered
    //  broken
    const int sense_threshold = 8000;

    // Number of consecative "broken" senses
    //  before we are confident the beam is broken
    const unsigned int consec_num = 2;

    // Consecative "broken" senses counter
    unsigned int consec_ctr = 0;

    RtosTimer _updateTimer;

    static void updateCallback(const void* instance);
};
