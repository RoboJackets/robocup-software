#include "rj_benchmarking/registry.hpp"

Registry::Registry() : rclcpp::Node{"rj_benchmarking"} {
    subscription_ = this->create_subscription<rj_msgs::msg::Latency>(
        "/registry", 100, std::bind(&Registry::topic_callback, this, std::placeholders::_1));
}

Registry::~Registry() {
    /*
        1. Make LatencyLogs Directory if not already there
        2. Make latency_curr-date_curr-time folder
        3. Make csvs for all robots
            3a. first row is labels
    */

    if (max_rows_ != 0) {
        std::string base_path{"./latency"};
        std::filesystem::create_directories(base_path);
        base_path += "/session_";
        base_path += get_curr_datetime();
        std::filesystem::create_directories(base_path);

        for (size_t i = 0; i < kNumShells; i++) {
            if (registry_[i].empty()) {
                continue;
            }

            std::string ss{};
            ss += "/robot_";
            ss += std::to_string(i);
            ss += ".csv";
            std::ofstream robot_csv {base_path + ss};
            print_data(robot_csv, i);
        }

        if (!registry_[kNumShells].empty())
        {
            std::string file_name {"/global.csv"};
            std::ofstream global_csv {base_path + file_name};
            print_data(global_csv, kNumShells);
        }
    }
}

void Registry::topic_callback(const rj_msgs::msg::Latency& msg) {
    if (msg.robot_id == -1)  // robot independent; will go at end
    {
        registry_[kNumShells][msg.label].push_back(msg.duration_ns);
        max_rows_ = std::max(max_rows_, static_cast<size_t>(
            registry_[kNumShells][msg.label].size()));
    } else {
        registry_[msg.robot_id][msg.label].push_back(msg.duration_ns);
        max_rows_ = std::max(max_rows_, static_cast<size_t>(
            registry_[msg.robot_id][msg.label].size()));
    }
}

std::string Registry::get_curr_datetime() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};

    localtime_r(&now_time, &tm_buf);

    std::ostringstream ss;
    ss << std::put_time(&tm_buf, "%Y-%m-%d_%H:%M:%S");
    return ss.str();
}

void Registry::print_data(std::ofstream& file, int registry_index) {
    // Print out labels
    for (const auto& [label, timestamps] : registry_[registry_index]) {
        file << label << ',';
    }
    file << '\n';

    // Print out data row by row
    for (size_t row = 0; row < max_rows_; ++row) {
        for (const auto& [label, timestamps] : registry_[registry_index]) {
            if (row >= timestamps.size()) {
                // sentinel value
                file << -1 << ',';
            } else {
                file << timestamps[row] << ',';
            }
        }
        file << '\n';
    }
}