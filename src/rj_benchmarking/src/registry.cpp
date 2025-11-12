#include "rj_benchmarking/registry.hpp"

Registry::Registry() : rclcpp::Node{"rj_benchmarking"}
{
    // SPDLOG_INFO("TESTING: Registry Built");
    subscription_ = this->create_subscription<rj_msgs::msg::Latency>(
            "/registry", 100, std::bind(&Registry::topic_callback, this,
                std::placeholders::_1));
}

Registry::~Registry()
{
    dump();
    // SPDLOG_INFO("TESTING: Registry Destroyed");
}

void Registry::topic_callback(const rj_msgs::msg::Latency &msg)
{
    registry_.at(msg.robot_id)[msg.label].push_back(msg.duration_ns);
    max_rows_ = std::max(max_rows_, static_cast<int>(registry_.at(msg.robot_id)[msg.label].size()));
}


void Registry::dump()
{
    // SPDLOG_INFO("TESTING: Dump Called");

    /*
        1. Make LatencyLogs Directory if not already there
        2. Make latency_curr-date_curr-time folder
        3. Make csv for Robot 1
            3a. first row is labels
        4. Make csvs for all robots
    */
    std::string base_path{ "./latency" };
    std::filesystem::create_directories(base_path);
    base_path += "/session_";
    base_path += get_curr_datetime();
    std::filesystem::create_directories(base_path);

    // std::ofstream output_file;
    // output_file.open(path_);
    for (int i = 0; i < 6; i++)
    {
        std::stringstream ss;
        ss << "/robot_" << i << ".csv";
        std::ofstream robot_csv{ base_path + ss.str() };

        for (int row = -1; row < max_rows_; row++)
        {
            if (row == -1)
            {
                for (std::pair<std::string, std::vector<uint64_t>> labels : registry_.at(i))
                {
                    robot_csv << labels.first << ',';
                }

                robot_csv << '\n';
            }

            for (std::pair<std::string, std::vector<uint64_t>> labels : registry_.at(i))
            {
                robot_csv << labels.second[row] << ',';
            }

            robot_csv << '\n';
        }

        
        // output_file << "Robot " << i << '\n';
        // for (auto& p : registry_[i])
        // {
        //     output_file << p.first << "     ";
        //     for (uint64_t e : p.second)
        //     {
        //         output_file << e << ", ";
        //     }
        //     output_file << "AVG: " << std::accumulate(p.second.begin(), p.second.end(), 0)
        //                                                                         / p.second.size();
        //     output_file << '\n';
        // }
    }
    // output_file.close();
}

std::string Registry::get_curr_datetime()
{
    std::time_t time = std::time({});
    char timeString[std::size("yyyy-mm-ddThh:mm:ssZ")];
    std::strftime(std::data(timeString), std::size(timeString),
                  "%FT%TZ", std::localtime(&time));
    std::string out{ timeString };
    return out;
}
