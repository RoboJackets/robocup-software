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
};
