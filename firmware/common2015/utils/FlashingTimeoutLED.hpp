#pragma once

#include <mbed.h>
#include <rtos.h>

/**
 * Flashes an LED as long as the timeout period hasn't elapsed since the last
 * call to renew().
 */
class FlashingTimeoutLED {
public:
    FlashingTimeoutLED(DigitalOut led)
        : _flashTimer(&flashCallback, osTimerPeriodic, this),
          _timeoutTimer(timeoutCallback, osTimerOnce, this),
          _led(led) {
        _flashTimer.start(80);
    }

    void renew() {
        _timedOut = false;
        _timeoutTimer.start(275);
    }

protected:
    /// Flash the LED on for a small duration, then back off
    void flash(uint32_t durationMsec = 15) {
        _led = !_led;
        Thread::wait(durationMsec);
        _led = !_led;
    }

private:
    static void flashCallback(const void* instance) {
        FlashingTimeoutLED* thiss = const_cast<FlashingTimeoutLED*>(
            reinterpret_cast<const FlashingTimeoutLED*>(instance));
        if (!thiss->_timedOut) thiss->flash();
    }

    static void timeoutCallback(const void* instance) {
        FlashingTimeoutLED* thiss = const_cast<FlashingTimeoutLED*>(
            reinterpret_cast<const FlashingTimeoutLED*>(instance));
        thiss->_timedOut = true;
    }

    RtosTimer _flashTimer;
    RtosTimer _timeoutTimer;
    DigitalOut _led;
    bool _timedOut = false;
};
