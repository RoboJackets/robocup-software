#include "rj_base_station/alive_robots_node.hpp"

namespace base_station {

AliveRobotsNode::AliveRobotsNode(): rclcpp::Node("alive_robots") {
    alive_robots_pub_ = create_publisher<rj_msgs::msg::AliveRobots>("alive_robots", 1);

    for (size_t i = 0; i < kNumShells; i++) {
        robot_status_subs_[i] = create_subscription<rj_msgs::msg::RobotStatus>(
            radio::topics::robot_status_topic(static_cast<int>(i)), rclcpp::QoS(1),
            // NOLINTNEXTLINE(performance-unnecessary-value-param)
            [this](const rj_msgs::msg::RobotStatus::SharedPtr robot_status) {
                this->last_updates_[robot_status->robot_id] = RJ::now();
            }
        );
        last_updates_[i] = RJ::Time::min();
    }

    tick_timer_= create_wall_timer(kTickPeriod, [this]{
        publish_alive_robots();
    });
}

void AliveRobotsNode::publish_alive_robots() {
    RJ::Time now = RJ::now();

    rj_msgs::msg::AliveRobots alive_robots;
    for (size_t i = 0; i < kNumShells; i++) {
        alive_robots.alive_robots[i] = now - last_updates_[i] < kDeathTimeout;
    }

    alive_robots_pub_->publish(alive_robots);
}

} // namespace base_station

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<base_station::AliveRobotsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}