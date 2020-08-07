#pragma once

#include <cmath>
#include <iostream>
#include <stdexcept>

const static bool kThrowDebugExceptions = true;

template <typename T>
inline int signum(T val) {
    return (0 < val) - (val <= 0);
}

inline void debug_log(const std::string& e) { std::cerr << e << std::endl; }

inline void debug_log(const std::exception& e) {
    std::cerr << e.what() << std::endl;
}

inline void debug_log_if(const std::string& e, bool condition) {
    if (condition) {
        debug_log(e);
    }
}

template <class T,
          typename std::enable_if<std::is_base_of<std::exception, T>::value,
                                  int>::type = 0>
inline void debug_throw(const T& e) {
    debug_log(e);
    if (kThrowDebugExceptions) {
        throw e;
    }
}

inline void debug_throw(const std::string& string) {
    debug_throw(std::runtime_error(string));
}

inline void debug_throw_if(const std::string& string, bool condition) {
    if (condition) {
        debug_throw(std::runtime_error(string));
    }
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

template <typename T, typename S>
void update_cache(T& value, const S& expected, bool* valid) {
    if (value != expected) {
        value = expected;
        *valid = false;
    }
}
