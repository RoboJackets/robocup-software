#pragma once
#include <algorithm>
#include <chrono>
#include <ctime>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace latency_benchmarking {

class Registry {
public:
  // Singleton Pattern
  static Registry& instance() {
    static Registry R;
    return R;
  }

  void record(const char* label, uint64_t ns) {

  }

  void dump() {

  }

  void clear() {}

private:
  // Constructor and creates callback that will dump latency statistics at program end
  Registry() {
    std::atexit([] {Registry::instance.dump();});
  }

  const time_t now = std::time(nullptr);
  std::tm* localTime = std::localtime(&now);
  std::string path_ = "../log/latency_" + localTime->tm_year + 1900 + "_" + localTime->tm_mon + 1 + "_" + localTime->tm_mday


}


} // namespace latency_benchmarking