#include "rj_benchmarking/timer.hpp"

Timer::Timer(const std::string& label, uint8_t robot_id)
    : label_(label), robot_id_(robot_id), start_(std::chrono::steady_clock::now()) {}

Timer::~Timer() {
    uint64_t time = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                              std::chrono::steady_clock::now() - start_)
                                              .count());
    RegistryPublisher::getInstance()->publish(label_, robot_id_, time);
}
