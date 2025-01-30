#include "rj_utils/logging.hpp"

#include <rclcpp/logging.hpp>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>

namespace rj_utils {
/**
 * @brief Sink for spdlog that passes the formatted message along to ros2.
 */
template <typename Mutex>
class Ros2Sink final : public spdlog::sinks::base_sink<Mutex> {
protected:
    // NOLINTNEXTLINE
    void sink_it_(const spdlog::details::log_msg& msg) final;

    // NOLINTNEXTLINE
    void flush_() final{};
};

template <typename Mutex>
// NOLINTNEXTLINE
void Ros2Sink<Mutex>::sink_it_(const spdlog::details::log_msg& msg) {
    spdlog::memory_buf_t formatted;
    spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);

    const std::string logger_name(msg.logger_name.begin(), msg.logger_name.end());
    const rclcpp::Logger logger = rclcpp::get_logger(logger_name);
    const auto string = fmt::to_string(formatted);
    switch (msg.level) {
        case spdlog::level::trace:
        case spdlog::level::debug:
            RCLCPP_DEBUG(logger, string.c_str());  // NOLINT
            break;
        case spdlog::level::info:
            RCLCPP_INFO(logger, string.c_str());  // NOLINT
            break;
        case spdlog::level::warn:
            RCLCPP_WARN(logger, string.c_str());  // NOLINT
            break;
        case spdlog::level::err:
            RCLCPP_ERROR(logger, string.c_str());  // NOLINT
            break;
        case spdlog::level::critical:
            RCLCPP_FATAL(logger, string.c_str());  // NOLINT
            break;
        case spdlog::level::off:
            break;
    }
}

void set_spdlog_default_ros2(const std::string& logger_name) {
    auto ros2_sink = std::make_shared<Ros2Sink<spdlog::details::null_mutex>>();
    auto ros2_logger = std::make_shared<spdlog::logger>(logger_name, std::move(ros2_sink));

    // Also include the source filename and line number, but remove all the other level / timestamp
    // information as that is provided by ros2.
    ros2_logger->set_pattern("[%s:%#] %v");
    spdlog::set_default_logger(std::move(ros2_logger));
}
}  // namespace rj_utils
