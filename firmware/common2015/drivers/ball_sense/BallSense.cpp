#include "BallSense.hpp"

BallSense::BallSense(DigitalOut emitter, AnalogIn detector)
    : emitter_pin(emitter),
      detector_pin(detector),
      _updateTimer(this, &BallSense::update_ball_sensor, osTimerPeriodic) {
    emitter_pin = false;
}

void BallSense::update_ball_sensor() {
    if (emitter_on) {
        // Update value
        int sense_light = detector_pin.read_u16();

        // Shutoff light
        emitter_on = false;
        emitter_pin.write(0);

        // Possible break in beam
        if (std::abs(sense_light - sense_dark) < sense_threshold) {
            consec_ctr++;
        } else {
            consec_ctr = 0;
        }
        printf("{'light': %d, 'dark': %d, 'diff': %d, 'ball': %s},\r\n", sense_light,
        sense_dark, std::abs(sense_light - sense_dark), have_ball() ? "True" : "False");
    } else  // Emitter off
    {
        // Update value
        sense_dark = detector_pin.read_u16();

        // Turn on light
        emitter_on = true;
        emitter_pin.write(1);
    }
}

bool BallSense::have_ball() { return consec_ctr < consec_num; }
