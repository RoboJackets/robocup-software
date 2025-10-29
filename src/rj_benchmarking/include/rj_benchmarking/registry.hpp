#include <string>
#include <unordered_map>
#include <array>
#include <vector>
#include <numeric>
#include <fstream>
#include <spdlog/spdlog.h>

#include <cstdint>

namespace benchmarking {

class Registry {
public:
    // Singleton Pattern
    static Registry& instance() {
        static Registry R;
        return R;
    }

    void record(std::string label, uint64_t time, int8_t robot_id);

    void dump();

  // Constructor and creates callback that will dump latency statistics at program end
    Registry();
    ~Registry();

private:
    std::string path_ = "../../log/latency.txt";
    std::array<std::unordered_map<std::string, std::vector<uint64_t>>, 6> registry_;
};
} // namespace benchmarking