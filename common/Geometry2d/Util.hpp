
#pragma once

#include <cmath>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif
#ifndef M_PI_2
#define M_PI_2 1.570796326794897
#endif
#ifndef M_E
#define M_E 2.718281828459045
#endif

constexpr float DegreesToRadians(float x) {
    return (x)*M_PI / 180.f;
}

constexpr float RadiansToDegrees(float x) {
    return (x)*180.f / M_PI;
}

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
