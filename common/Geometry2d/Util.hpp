#pragma once

#include <math.h>

const float DegreesToRadians = M_PI / 180.0;
const float RadiansToDegrees = 180.0 / M_PI;

template <typename T>
static inline T sign(T f) {
    if (f < T(0)) {
        return -1;
    } else if (f > T(0)) {
        return 1;
    } else {
        return 0;
    }
}
