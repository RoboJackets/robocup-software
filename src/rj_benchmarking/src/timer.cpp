#include "rj_benchmarking/timer.hpp"
#include "rj_benchmarking/registry.hpp"

Timer::Timer(std::string label, int8_t robot_id) : label_(label), robot_id_(robot_id), start_(std::chrono::steady_clock::now()) {
  SPDLOG_INFO("HEY DONT LOOK AT ME");
}

Timer::~Timer() {
  SPDLOG_INFO("hey look at me");
  uint64_t time = static_cast<uint64_t>(std::chrono::
  duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start_).count());
  Registry::getInstance()->record(label_, time, robot_id_);
}
