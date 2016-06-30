#pragma once

#include <mbed.h>

class HackedKickerBoard {
public:
    HackedKickerBoard(DigitalOut kickLine)
        : _kickLine(kickLine),
          _kickTimer(this, &HackedKickerBoard::_stopKicking, osTimerOnce) {
            // ensure kicker is off
            _kickLine = 0;
          }

    void kick(uint8_t power) {
        // TODO: convert from power to kick duration better
        _kickTimer.start(power);

        _kickLine = 1;
    }

protected:
    void _stopKicking() { _kickLine = 0; }

private:
    DigitalOut _kickLine;
    RtosTimerHelper _kickTimer;
};
