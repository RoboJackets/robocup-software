#pragma once

#include <cmath>
#include <iostream>
#include <stdexcept>

const static bool THROW_DEBUG_EXCEPTIONS = true;

template <typename T>
inline int signum(T val) {
    return (0 < val) - (val <= 0);
}

inline void debugLog(const std::string& e) { std::cerr << e << std::endl; }

inline void debugLog(const std::exception& e) {
    std::cerr << e.what() << std::endl;
}

inline void debugLogIf(const std::string& e, bool condition) {
    if (condition) {
        debugLog(e);
    }
}

template <class T,
          typename std::enable_if<std::is_base_of<std::exception, T>::value,
                                  int>::type = 0>
inline void debugThrow(const T& e) {
    debugLog(e);
    if (THROW_DEBUG_EXCEPTIONS) {
        throw e;
    }
}

inline void debugThrow(const std::string& string) {
    debugThrow(std::runtime_error(string));
}

inline void debugThrowIf(const std::string& string, bool condition) {
    if (condition) {
        debugThrow(std::runtime_error(string));
    }
}

/**
 * @brief Restricts the given angle to be between pi and -pi
 *
 * @param a An angle in radians
 * @return An equivalent angle in radians restricted to [-pi, pi]
 */
template <typename T>
static inline T fixAngleRadians(T a) {
    return remainder(a, 2 * M_PI);
}

template <typename T>
inline T applyLowPassFilter(const T& oldValue, const T& newValue, double gain) {
    return gain * newValue + (1 - gain) * oldValue;
}

template <typename T>
void update_cache(T& value, const T& expected, bool* valid) {
    if (value != expected) {
        value = expected;
        *valid = false;
    }
}
