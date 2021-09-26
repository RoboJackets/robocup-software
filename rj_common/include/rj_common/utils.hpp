#pragma once

#include <cmath>
#include <stdexcept>

#include <spdlog/spdlog.h>

template <typename T>
inline int signum(T val) {
    return (0 < val) - (val <= 0);
}

/**
 * @brief Restricts the given angle to be between pi and -pi
 *
 * @param a An angle in radians
 * @return An equivalent angle in radians restricted to [-pi, pi]
 */
template <typename T>
static inline T fix_angle_radians(T a) {
    return remainder(a, 2 * M_PI);
}

template <typename T>
inline T apply_low_pass_filter(const T& old_value, const T& new_value, double gain) {
    return gain * new_value + (1 - gain) * old_value;
}

template <typename T>
inline T lerp(const T& a, const T& b, double factor) {
    return (1 - factor) * a + factor * b;
}

template <typename T, typename S>
void update_cache(T& value, const S& expected, bool* valid) {
    if (value != expected) {
        value = expected;
        *valid = false;
    }
}
