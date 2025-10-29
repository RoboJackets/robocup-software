#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

namespace benchmarking {
    class Benchmarking : public rclcpp::Node {
    public:
        Benchmarking();
        ~Benchmarking();

    private:
        // std::string path_ = "../../log/latency.txt";
        // std::array<std::unordered_map<std::string, std::vector<uint64_t>>, 6> registry_;
    };
}