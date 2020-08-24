#pragma once

#include <stdexcept>
#include <string>

#include <spdlog/spdlog.h>

namespace rj_utils {
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
    throw std::runtime_error(fmt::format(__VA_ARGS__));

}  // namespace rj_utils
