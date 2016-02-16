#pragma once

#include <cmath>

#define DegreesToRadians(x) (float)((x)*M_PI / 180.f)
#define RadiansToDegrees(x) (float)((x)*180.f / M_PI)

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

static const float FLOAT_EPSILON = 0.00001;
static bool nearlyEqual(float a, float b) {
    return std::fabs(a - b) < FLOAT_EPSILON;
}
