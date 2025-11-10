#pragma once

#include <string>
#include <unordered_map>
#include <array>
#include <vector>
#include <numeric>
#include <fstream>
#include <cstdint>

#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>
#include <rj_msgs/msg/latency.hpp>


class Registry : public rclcpp::Node
{
public:
    // Singleton Pattern
    // static Registry* getInstance()
    // {
    //     if (instance == nullptr)
    //     {
    //         instance = new Registry();
    //     }

    //     return instance;
    // }

    // void record(std::string label, uint64_t time, int8_t robot_id);

    // void dump();
    Registry();
    ~Registry();

private:
    // static Registry* instance;

    // Private Constructor
    // Registry();

    // Delete Copy Constructor and Assignment
    // Registry(const Registry& other) = delete;
    // Registry& operator=(const Registry& other) = delete;

    void topic_callback(const rj_msgs::msg::Latency &msg);
    void dump();

    std::string path_ { "log/latency.txt" };
    
    // registry[robot_id][label] -> latency sampling
    std::array<std::unordered_map<std::string, std::vector<uint64_t>>, 6> registry_;
    rclcpp::Subscription<rj_msgs::msg::Latency>::SharedPtr subscription_;
};
