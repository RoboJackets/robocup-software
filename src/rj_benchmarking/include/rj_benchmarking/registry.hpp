#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_constants/constants.hpp>
#include <rj_msgs/msg/latency.hpp>

// Set RobotId to -1 for Global Profiling
class Registry : public rclcpp::Node {
public:
    Registry();
    ~Registry();

private:
    void topic_callback(const rj_msgs::msg::Latency& msg);

    std::string get_curr_datetime();
    void print_data(std::ofstream& file, int registry_index);

    std::array<std::unordered_map<std::string, std::vector<uint64_t>>, kNumShells + 1> registry_{};
    rclcpp::Subscription<rj_msgs::msg::Latency>::SharedPtr subscription_{};
    size_t max_rows_{};
};
