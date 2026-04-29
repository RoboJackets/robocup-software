#include "rj_base_station/alive_robots_node.hpp"

#include <rj_constants/constants.hpp>

namespace base_station {

AliveRobotsNode::AliveRobotsNode() : Node("alive_robots") {
    declare_parameter<int64_t>("death_timeout_ms", 500);
    death_timeout_ = std::chrono::milliseconds(get_parameter("death_timeout_ms").as_int());

    alive_robots_pub_ =
        create_publisher<rj_msgs::msg::AliveRobots>("/radio/alive_robots", rclcpp::QoS(1));

    for (size_t i = 0; i < kNumShells; i++) {
        robot_status_subs_[i] = create_subscription<rj_msgs::msg::RobotStatus>(
            fmt::format("/radio/robot_status/robot_{}", i), rclcpp::QoS(1),
            // NOLINTNEXTLINE(performance-unnecessary-value-param)
            [this](const rj_msgs::msg::RobotStatus::SharedPtr robot_status) {
                this->last_updates_[robot_status->robot_id] = std::chrono::system_clock::now();
            });
        last_updates_[i] = std::chrono::system_clock::time_point::min();
    }

    tick_timer_ = create_wall_timer(kTickPeriod, [this] { publish_alive_robots(); });

    parameter_callback_handle_ =
        add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters)
                                           -> rcl_interfaces::msg::SetParametersResult {
            rcl_interfaces::msg::SetParametersResult result;
            for (const auto& param : parameters) {
                if (param.get_name() == "death_timeout_ms") {
                    death_timeout_ = std::chrono::milliseconds(param.as_int());
                }
            }
            result.successful = true;
            return result;
        });
}

void AliveRobotsNode::publish_alive_robots() {
    auto now = std::chrono::system_clock::now();

    rj_msgs::msg::AliveRobots alive_robots;
    for (size_t i = 0; i < kNumShells; i++) {
        alive_robots.alive_robots[i] = now - last_updates_[i] < death_timeout_;
    }

    alive_robots_pub_->publish(alive_robots);
}

}  // namespace base_station

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<base_station::AliveRobotsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
