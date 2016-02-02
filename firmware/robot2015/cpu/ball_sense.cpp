#include "ball_sense.hpp"
#include "robot.hpp"

ball_sense::ball_sense() {
    emitter_pin = new DigitalOut(RJ_BALL_EMITTER);
    detector_pin = new AnalogIn(RJ_BALL_DETECTOR);

    emitter_pin->write(0);
}

void ball_sense::update_ball_sensor() {
    if (emitter_on) {
        // Update value
        sense_light = detector_pin->read_u16();

        // Shutoff light
        emitter_on = false;
        emitter_pin->write(0);

        // Possible break in beam
        if (sense_light - sense_dark > sense_threshold) {
            consec_ctr++;
        } else {
            consec_ctr = 0;
        }
    } else  // Emitter off
    {
        // Update value
        sense_dark = detector_pin->read_u16();

        // Turn on light
        emitter_on = true;
        emitter_pin->write(1);
    }
}

bool ball_sense::have_ball() { return consec_ctr >= consec_num; }
