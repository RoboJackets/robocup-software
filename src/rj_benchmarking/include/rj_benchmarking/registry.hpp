#pragma once

#include <rj_msgs/msg/latency.hpp>

#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_map>
#include <array>
#include <vector>
#include <numeric>
#include <fstream>
#include <cstdint>
#include <filesystem>
#include <ctime>
#include <iterator>

class Registry : public rclcpp::Node
{
public:
    Registry();
    ~Registry();

private:
    void topic_callback(const rj_msgs::msg::Latency &msg);
    void dump();

    std::string get_curr_datetime();
    
    // registry[robot_id][label] -> latency sampling
    std::array<std::unordered_map<std::string, std::vector<uint64_t>>, 6> registry_{};
    rclcpp::Subscription<rj_msgs::msg::Latency>::SharedPtr subscription_{};
    int max_rows_{};
};
