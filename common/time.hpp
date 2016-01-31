#pragma once

#include <sys/time.h>

namespace RJ {

/// type for storing time in microseconds
typedef uint64_t Time;

/** returns the local system timestamp in microseconds */
static inline Time timestamp() {
    struct timeval time;
    gettimeofday(&time, nullptr);

    return (Time)time.tv_sec * 1000000 + (Time)time.tv_usec;
}

/// Converts a decimal number of seconds to an integer timestamp in microseconds
static inline RJ::Time SecsToTimestamp(double secs) {
    return secs * 1000000.0f;
}

/// Converts an integer timestamp in microseconds to a decimal number of seconds
static inline float TimestampToSecs(RJ::Time timestamp) {
    return (float)(timestamp / 1000000.0f);
}

}  // namespace RJ
