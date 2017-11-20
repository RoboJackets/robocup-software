
#pragma once

#include <Utils.hpp>
#include <time.hpp>

/**
 * This is a simple timeout timer.
 *
 * Initialize it with a given duration,
 * then query it repeatedly to see if the duration is over.
 */
class Timeout {
public:
    Timeout(RJ::Seconds interval = 0ms) : _interval(seconds) { reset(); }

    void reset() { _startTime = RJ::now(); }

    void setIntervalInSeconds(RJ::Seconds seconds) { _interval = seconds; }

    bool isTimedOut() { return RJ::now() - _startTime > _interval; }

private:
    RJ::Seconds _interval;
    RJ::Time _startTime;
};
