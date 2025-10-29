#include "rj_benchmarking/benchmarking.hpp"
#include "rj_benchmarking/registry.hpp"

namespace benchmarking {
    Benchmarking::Benchmarking() : rclcpp::Node{"rj_benchmarking"}
    {
        SPDLOG_INFO("TESTING: Benchmarking Built");
        Registry();
    }

    Benchmarking::~Benchmarking()
    {
        SPDLOG_INFO("TESTING: Benchmarking Destroyed");
    }
}