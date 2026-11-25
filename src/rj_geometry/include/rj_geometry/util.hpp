#pragma once

#include <cmath>
#include <math.h>



constexpr float degrees_to_radians(float x) {
    return (x) * static_cast<float>(M_PI) / 180.f;
}

constexpr float radians_to_degrees(float x) {
    return (x)*180.f / static_cast<float>(M_PI);
}

static bool nearly_equal(double a, double b, double tolerance = 1e-4) {
    return std::fabs(a - b) < tolerance;
}
