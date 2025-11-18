#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <iomanip>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/msg/latency.hpp>
#include <rj_constants/constants.hpp>

class Registry : public rclcpp::Node {
public:
    Registry();
    ~Registry();

private:
    void topic_callback(const rj_msgs::msg::Latency& msg);

    std::string get_curr_datetime();

    // registry[robot_id][label] -> latency sampling
    std::array<std::unordered_map<std::string, std::vector<uint64_t>>, kNumShells> registry_{};
    rclcpp::Subscription<rj_msgs::msg::Latency>::SharedPtr subscription_{};
    size_t max_rows_{};
};
