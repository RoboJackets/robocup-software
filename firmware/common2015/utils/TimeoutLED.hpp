#pragma once

#include <mbed.h>
#include <rtos.h>
#include <RtosTimerHelper.hpp>

/**
 * Utility class that lights LEDs as long as the timeout period hasn't elapsed
 * since the last call to renew().  This class provides an update() method that
 * subclasses override to actual light/flash/strobe the LED(s).  This class just
 * handles the timing aspects.
 */
class TimeoutLED {
public:
    TimeoutLED(uint32_t updateInterval = 80, uint32_t timeoutInterval = 275)
        : _updateTimer(this, &TimeoutLED::_update, osTimerPeriodic),
          _timeoutTimer(this, &TimeoutLED::_timeout, osTimerOnce),
          _timeoutInterval(timeoutInterval) {
        _updateTimer.start(updateInterval);
        renew();
    }

    void renew() {
        _timedOut = false;
        _timeoutTimer.start(_timeoutInterval);
    }

    /**
     * Subclasses must override this method to handle the actual LED lighting,
     * etc.
     */
    virtual void update() = 0;

private:
    void _timeout() { _timedOut = true; }

    void _update() {
        if (!_timedOut) update();
    }

    RtosTimerHelper _updateTimer;
    RtosTimerHelper _timeoutTimer;
    bool _timedOut = false;
    uint32_t _timeoutInterval;
};

/**
 * Flashes a single LED as long as the timer hasn't expired.
 */
class FlashingTimeoutLED : public TimeoutLED {
public:
    FlashingTimeoutLED(DigitalOut led) : _led(led) {}

    /// Flash the LED on for a small duration, then back off
    virtual void update() {
        _led = !_led;
        Thread::wait(15);
        _led = !_led;
    }

private:
    DigitalOut _led;
};

/**
 * Strobes an array of leds as long as the timer hasn't expired.
 */
template <size_t NUM_LEDS>
class StrobingTimeoutLEDs : public TimeoutLED {
public:
    StrobingTimeoutLEDs(std::array<DigitalOut, NUM_LEDS> leds,
                        uint32_t updateInterval, uint32_t timeoutInterval)
        : TimeoutLED(updateInterval, timeoutInterval), _leds(leds) {}

    /// Strobe the leds
    void update() {
        if (_dir) {
            for (size_t i = 0; i < _leds.size(); ++i) {
                _leds[i] = !_leds[i];
                Thread::wait(17);
                _leds[i] = !_leds[i];
            }
        } else {
            for (size_t i = _leds.size(); i > 0; --i) {
                _leds[i - 1] = !_leds[i - 1];
                Thread::wait(17);
                _leds[i - 1] = !_leds[i - 1];
            }
        }

        _dir = !_dir;
    }

private:
    std::array<DigitalOut, NUM_LEDS> _leds;

    // Direction we're currently strobing
    bool _dir;
};
