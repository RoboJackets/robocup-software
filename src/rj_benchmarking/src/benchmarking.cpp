#include "rj_benchmarking/benchmarking.hpp"
#include "rj_benchmarking/registry.hpp"

Benchmarking::Benchmarking() : rclcpp::Node{"rj_benchmarking"}
{
    SPDLOG_INFO("TESTING: Benchmarking Built");
    Registry::getInstance();
}

Benchmarking::~Benchmarking()
{
    SPDLOG_INFO("TESTING: Benchmarking Destroyed");
}
