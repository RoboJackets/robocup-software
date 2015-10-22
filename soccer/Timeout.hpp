
#pragma once

#include <Utils.hpp>

/**
 * This is a simple timeout timer.
 *
 * Initialize it with a given duration,
 * then query it repeatedly to see if the duration is over.
 */
class Timeout {
public:
    Timeout(float seconds = 0) {
        setIntervalInSeconds(seconds);
        reset();
    }

    void reset() { _startTime = RJ::timestamp(); }

    void setIntervalInSeconds(float seconds) {
        _interval = (RJ::Time)(seconds * 1000.0f);
    }

    void setIntervalInMilliseconds(RJ::Time ms) { _interval = ms; }

    bool isTimedOut() { return RJ::timestamp() - _startTime > _interval; }

private:
    RJ::Time _interval;
    RJ::Time _startTime;
};
