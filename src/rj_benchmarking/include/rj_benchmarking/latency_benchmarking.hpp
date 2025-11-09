#pragma once

#include <string>
#include <unordered_map>
#include <array>
#include <vector>
#include <iostream>
#include <chrono>
#include <numeric>
#include <fstream>
#include <spdlog/spdlog.h>

#include <cstdint>

namespace latency_benchmarking {

class Registry {
public:
  // // Singleton Pattern
  // static Registry& instance() {
  //   static Registry R;
  //   return R;
  // }

  // void record(std::string label, uint64_t time, int8_t robot_id) {
  //   registry_.at(robot_id)[label].push_back(time);
  // }

  // void dump() {
  //   SPDLOG_INFO("testing testing testing");
  //   std::ofstream output_file;
  //   output_file.open(path_);
  //   for (int i = 0; i < 6; i++) {
  //     output_file << "Robot " << i << "\n";
  //     for (auto& p : registry_[i]) {
  //         output_file << p.first << "     ";
  //         for (uint64_t e : p.second) {
  //             output_file << e << ", ";
  //         }
  //         output_file << "AVG: " << std::accumulate(p.second.begin(), p.second.end(), 0) / p.second.size();
  //         output_file << "\n";
  //     }
  //   }
  //   output_file.close();
  // }

  // void clear() {
  //   registry_.clear();
  // }

// private:
  // Constructor and creates callback that will dump latency statistics at program end
  Registry() {
    SPDLOG_INFO("TESTING TESTING TESTING");
    printf("TESTING FROM PRINTFFFF");
  }
  ~Registry() {
    // Registry::instance().dump();
    SPDLOG_INFO("DESTRUCTOR HAS BEEN CALLED TESTING");
  }

  // void lol() {
  //   printf("lol");
  // }

  std::string path_ = "../../log/latency.txt";
  std::array<std::unordered_map<std::string, std::vector<uint64_t>>, 6> registry_;
};

class Timer{

public:
  Timer(std::string label, int8_t robot_id) : label_(label), robot_id_(robot_id), start_(std::chrono::steady_clock::now()) {
    SPDLOG_INFO("HEY DONT LOOK AT ME");
  }
  ~Timer() {
    SPDLOG_INFO("hey look at me");
    uint64_t time = static_cast<uint64_t>(std::chrono::
      duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start_).count());
    // Registry::instance().record(label_, time, robot_id_);
  }
private:
const std::chrono::steady_clock::time_point start_;
std::string label_;
int8_t robot_id_;
};


} // namespace latency_benchmarking
