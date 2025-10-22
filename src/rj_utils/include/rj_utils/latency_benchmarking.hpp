#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <chrono>
#include <numeric>
#include <fstream>

#include <cstdint>

namespace latency_benchmarking {

class Registry {
public:
  // Singleton Pattern
  static Registry& instance() {
    static Registry R;
    return R;
  }

  void record(std::string label, uint64_t time) {
    registry_[label].push_back(time);
  }

  void dump() {
    std::ofstream output_file;
    output_file.open(path_);
    for (auto& p : registry_) {
        output_file << p.first << "     ";
        for (uint64_t e : p.second) {
            output_file << e << " ";
        }
        output_file << "AVG: " << std::accumulate(p.second.begin(), p.second.end(), 0) / p.second.size();
        output_file << "\n";
    }
    output_file.close();
  }

  void clear() {
    registry_.clear();
  }

private:
  // Constructor and creates callback that will dump latency statistics at program end
  Registry() {}
  ~Registry() {
    Registry::instance().dump();
  }

  std::string path_ = "../../log/latency.txt";
  std::unordered_map<std::string, std::vector<uint64_t>> registry_;
};

class Timer{

public:
  Timer(std::string label) : label_(label), start_(std::chrono::steady_clock::now()) {}
  ~Timer() {
    uint64_t time = static_cast<uint64_t>(std::chrono::
      duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start_).count());
    Registry::instance().record(label_, time);
  }
private:
const std::chrono::steady_clock::time_point start_;
std::string label_;
};


} // namespace latency_benchmarking