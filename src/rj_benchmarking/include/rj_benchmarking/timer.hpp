#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/msg/latency.hpp>

#include "rj_benchmarking/registry_publisher.hpp"

class Timer {
public:
    Timer(const std::string& label, uint8_t robot_id);
    ~Timer();

private:
    std::string label_;
    std::uint8_t robot_id_;
    const std::chrono::steady_clock::time_point start_;
};
