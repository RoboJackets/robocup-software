#include <string>
#include <chrono>

#include <spdlog/spdlog.h>

#include <cstdint>

class Timer{

public:
  Timer(std::string label, std::int8_t robot_id);
  ~Timer();

private:
  const std::chrono::steady_clock::time_point start_;
  std::string label_;
  std::int8_t robot_id_;
};
