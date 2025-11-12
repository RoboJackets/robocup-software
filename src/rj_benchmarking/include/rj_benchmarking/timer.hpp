#pragma once

#include "rj_benchmarking/registry_publisher.hpp"
#include <rj_msgs/msg/latency.hpp>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <string>
#include <chrono>
#include <cstdint>

class Timer
{

public:
  Timer(std::string label, std::int8_t robot_id);
  ~Timer();

private:
  const std::chrono::steady_clock::time_point start_{};
  std::string label_{};
  std::int8_t robot_id_{};
};
