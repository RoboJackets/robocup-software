#pragma once

#include <string>

namespace rj_utils {
/**
 * Sets the default spdlog logger to use Ros2Sink.
 */
void set_spdlog_default_ros2(const std::string& logger_name);

}  // namespace rj_utils
