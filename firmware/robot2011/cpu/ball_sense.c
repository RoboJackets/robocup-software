#include <board.h>

#include "main.h"
#include "status.h"
#include "timer.h"
#include "ball_sense.h"
#include "adc.h"

// The maximum amount of ambient light we will tolerate.
// FIXME - This number is arbitrary and not based on experiment
static const int Dazzle_Threshold = 500;

// The minimum difference between light and dark measurements that indicates the
// beam is not broken.
// FIXME - This number is arbitrary and not based on experiment
static const int Unbroken_Beam = 500;  // was 700

// How many consecutive cycles the detector input must be stuck at full scale
// for the
// detector to be declared open-circuited.
static const int Det_Open_Time = 100;

static const int Det_Open_Threshold = 3;

int have_ball;
int ball_sense_light, ball_sense_dark;

// This is incremented for each cycle in which the detector input is at full
// scale.
// If the input stays at full scale, the detector has probably failed open.  A
// working detector normally passes some current.
int det_open_count;

void update_ball_sensor() {
    static int wait = 0;

    // Only run every other cycle.
    // ADC conversions are started just before this is called,
    // and we want to wait long enough after toggling the LED to let the
    // detector
    // input settle (due to the large pull-up resistor).
    wait = !wait;
    if (wait) {
        return;
    }

    // Update the ball sensor and toggle its LED.
    // Invert the data so increasing values indicate increasing light.
    if (LED_IS_ON(BALL_LED)) {
        // LED was on
        ball_sense_light = 0x3ff - adc[4];

        // Check for emitter failure
        //
        // While the output is low, switch it to input.  If the LED is
        // connected, it will
        // quickly pull the pin high.  If the LED is open, the pin will stay
        // low.
        AT91C_BASE_PIOA->PIO_CODR = BALL_LED;
        AT91C_BASE_PIOA->PIO_ODR = BALL_LED;
        if (!(AT91C_BASE_PIOA->PIO_PDSR & BALL_LED)) {
            failures |= Fail_Ball_LED_Open;
        } else {
            failures &= ~Fail_Ball_LED_Open;
        }

        // Turn the LED off
        LED_OFF(BALL_LED);

        // Make the pin output again
        AT91C_BASE_PIOA->PIO_OER = BALL_LED;
    } else {
        // LED was off
        ball_sense_dark = 0x3ff - adc[4];
        LED_ON(BALL_LED);
    }

    // Check for detector failures and excessive ambient light
    failures &= ~(Fail_Ball_Det_Open | Fail_Ball_Det_Short | Fail_Ball_Dazzled);
    if (ball_sense_dark <= Det_Open_Threshold &&
        ball_sense_light <= Det_Open_Threshold) {
        // Detector may be open (the pullup resistor pulled the ADC input to
        // 3.3V)
        if (det_open_count >= Det_Open_Time) {
            failures |= Fail_Ball_Det_Open;
        } else {
            ++det_open_count;
        }
    } else {
        // The detector is working
        det_open_count = 0;
    }

    if (ball_sense_dark == 0x3ff && ball_sense_light == 0x3ff) {
        // Detector is shorted so the ADC input is fixed at GND
        failures |= Fail_Ball_Det_Short;
    } else if (ball_sense_dark > Dazzle_Threshold) {
        // Too much outside light
        failures |= Fail_Ball_Dazzled;
    }

    // Update have_ball and the ball status LED
    if (failures & Fail_Ball) {
        // The ball sensor is broken
        have_ball = 0;
    } else {
        // The ball sensor works, so determine if we have the ball
        have_ball = (ball_sense_light - ball_sense_dark) < Unbroken_Beam;
    }
}
