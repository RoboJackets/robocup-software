#include "rj_benchmarking/timer.hpp"

Timer::Timer(const std::string& label, uint8_t robot_id)
    : label_(label), robot_id_(robot_id), start_(ros_clock().now()) {}

Timer::~Timer() {
    rclcpp::Duration duration = ros_clock().now() - start_;
    uint64_t time_ns = static_cast<uint64_t>(duration.nanoseconds());
    RegistryPublisher::getInstance()->publish(label_, robot_id_, time_ns);
}
