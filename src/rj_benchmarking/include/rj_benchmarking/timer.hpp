#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/msg/latency.hpp>

#include "rj_benchmarking/registry_publisher.hpp"

// Lazy static ros_clock
static rclcpp::Clock& ros_clock() {
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    return clock;
}

class Timer {
public:
    Timer(const std::string& label, uint8_t robot_id);
    ~Timer();

private:
    const std::string label_;
    const std::uint8_t robot_id_;
    const rclcpp::Time start_;
};
