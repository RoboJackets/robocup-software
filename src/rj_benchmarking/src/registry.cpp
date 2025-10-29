#include "rj_benchmarking/registry.hpp"

namespace benchmarking {
    Registry::Registry() : rclcpp::Node{"rj_benchmarking"}
    {
        SPDLOG_INFO("TESTING: Registry Built");
    }

    Registry::~Registry()
    {
        SPDLOG_INFO("TESTING: Registry Destroyed");
    }
}