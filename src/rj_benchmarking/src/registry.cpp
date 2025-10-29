#include "rj_benchmarking/registry.hpp"

namespace benchmarking {
    Registry::Registry() {
        SPDLOG_INFO("TESTING: Registry Built");
    }

    Registry::~Registry() {
        Registry::instance().dump();
        SPDLOG_INFO("TESTING: Registry Destroyed");
    }

    void Registry::record(std::string label, uint64_t time, int8_t robot_id) {
        registry_.at(robot_id)[label].push_back(time);
    }

    void Registry::dump() {
        SPDLOG_INFO("TESTING: Dump Called");
        std::ofstream output_file;
        output_file.open(path_);
        for (int i = 0; i < 6; i++) {
            output_file << "Robot " << i << "\n";
            for (auto& p : registry_[i]) {
                output_file << p.first << "     ";
                for (uint64_t e : p.second) {
                    output_file << e << ", ";
                }
                output_file << "AVG: " << std::accumulate(p.second.begin(), p.second.end(), 0) / p.second.size();
                output_file << "\n";
            }
        }
        output_file.close();
    }
}