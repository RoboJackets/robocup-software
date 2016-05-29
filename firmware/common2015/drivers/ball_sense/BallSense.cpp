#include "BallSense.hpp"

BallSense::BallSense(DigitalOut emitter, AnalogIn detector)
    : emitter_pin(emitter), detector_pin(detector),
      _updateTimer(this, &BallSense::update_ball_sensor, osTimerPeriodic) {
  emitter_pin = false;
}

void BallSense::update_ball_sensor() {
  if (emitter_on) {
    // Update value
    int sense_light = detector_pin.read_u16();

    // Shutoff light
    emitter_on = false;

    // Possible break in beam
    if (sense_light - sense_dark > sense_threshold) {
      consec_ctr++;
    } else {
      consec_ctr = 0;
    }
  } else { // Emitter off
    // Update value
    if (!consec_ctr)
      sense_dark = detector_pin.read_u16();

    // Turn on light
    emitter_on = true;
  }

  emitter_pin.write(emitter_on);

  // printf("%d ", detector_pin.read_u16());
  // fflush(stdout);
}

bool BallSense::have_ball() {
  return detector_pin.read_u16() > sense_threshold;
}
