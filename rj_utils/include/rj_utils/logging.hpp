#pragma once

#include <stdexcept>
#include <string>

#include <spdlog/spdlog.h>

namespace rj_utils {
const static bool kThrowDebugExceptions = true;

/**
 * @brief Sets the default spdlog logger to use Ros2Sink.
 */
void set_spdlog_default_ros2(const std::string& logger_name);

/**
 * @brief Convenience function for spdlog::critical, then throwing a std::runtime_error;
 */
template <typename... Args>
inline void fatal_throw(spdlog::string_view_t fmt, const Args&... args) {
    spdlog::critical(fmt, args...);
    throw std::runtime_error(fmt::format(fmt, args...));
}

/**
 * @brief Macro for fatal errors, logs a critical error + throwing a std::runtime_error.
 */
#define FATAL_THROW(...)          \
    SPDLOG_CRITICAL(__VA_ARGS__); \
    throw std::runtime_error(fmt::format(__VA_ARGS__))

inline void debug_log(const std::string& e) { spdlog::debug(e); }

inline void debug_log(const std::exception& e) { spdlog::debug(e.what()); }

inline void debug_log_if(const std::string& e, bool condition) {
    if (condition) {
        debug_log(e);
    }
}

template <class T,
          typename std::enable_if<std::is_base_of<std::exception, T>::value, int>::type = 0>
inline void debug_throw(const T& e) {
    debug_log(e);
    if (kThrowDebugExceptions) {
        throw e;
    }
}

inline void debug_throw(const std::string& string) { debug_throw(std::runtime_error(string)); }

inline void debug_throw_if(const std::string& string, bool condition) {
    if (condition) {
        debug_throw(std::runtime_error(string));
    }
}

}  // namespace rj_utils
