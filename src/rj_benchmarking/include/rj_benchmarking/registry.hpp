#pragma once

#include <array>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/msg/latency.hpp>

class Registry : public rclcpp::Node {
public:
    Registry();
    ~Registry();

private:
    void topic_callback(const rj_msgs::msg::Latency& msg);
    void dump();

    std::string get_curr_datetime();

    // registry[robot_id][label] -> latency sampling
    std::array<std::unordered_map<std::string, std::vector<uint64_t>>, 6> registry_{};
    rclcpp::Subscription<rj_msgs::msg::Latency>::SharedPtr subscription_{};
    int max_rows_{};
};
