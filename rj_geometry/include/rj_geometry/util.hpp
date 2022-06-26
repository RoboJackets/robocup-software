
#pragma once

#include <algorithm>
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

constexpr float degrees_to_radians(float x) {
    return (x) * static_cast<float>(M_PI) / 180.f;
}

constexpr float radians_to_degrees(float x) {
    return (x)*180.f / static_cast<float>(M_PI);
}

template <typename T>
static inline T sign(T f) {
    if (f < T(0)) {
        return -1;
    }
    if (f > T(0)) {
        return 1;
    }
    return 0;
}

// TODO(1485): Make this smaller once we figure out why test are failing in O3.
static bool nearly_equal(float a, float b, float tolerance = 1e-4) {
    return std::fabs(a - b) < tolerance;
}
