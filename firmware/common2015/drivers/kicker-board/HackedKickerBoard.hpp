#pragma once

#include <mbed.h>

class HackedKickerBoard {
public:

    // time to wait between kicks, in microseconds
    const uint32_t MIN_CHARGE_TIME = 1.5 * 1e6;

    HackedKickerBoard(DigitalOut kickLine)
        : _kickLine(kickLine),
          _kickTimer(this, &HackedKickerBoard::_stopKicking, osTimerOnce) {
            // ensure kicker is off
            _kickLine = 0;
            _lastKickTime = 0;
          }

    void kick(uint8_t power) {
        uint32_t t = us_ticker_read();

        // don't do anything - it hasn't charged enough since the last kick
        if (t - _lastKickTime < MIN_CHARGE_TIME) {
            return;
        }

        _lastKickTime = t;

        // power = 255 corresponds to 8ms kick time.  Everything lower is linearly scaled
        uint8_t time = (float)power / 255.0f * 8.0f;
        _kickTimer.start(time);

        _kickLine = 1;
    }

protected:
    void _stopKicking() { _kickLine = 0; }

private:
    DigitalOut _kickLine;
    RtosTimerHelper _kickTimer;
    uint32_t _lastKickTime;
};
