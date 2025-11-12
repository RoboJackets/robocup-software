#include "rj_benchmarking/timer.hpp"

Timer::Timer(std::string label, int8_t robot_id) : label_(label),
    robot_id_(robot_id), start_(std::chrono::steady_clock::now())
{
    SPDLOG_INFO("Testing: Timer Created " + label);
}

Timer::~Timer()
{
    SPDLOG_INFO("Testing: Timer Destroyed");
    uint64_t time = static_cast<uint64_t>(std::chrono::
        duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()
        - start_).count());
    
    RegistryPublisher::getInstance()->publish(label_, robot_id_, time);
}
