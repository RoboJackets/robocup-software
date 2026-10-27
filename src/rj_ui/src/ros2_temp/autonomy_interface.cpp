#include "rj_ui/ros2_temp/autonomy_interface.hpp"

namespace ros2_temp {

AutonomyInterface::AutonomyInterface(Context* context, rclcpp::Executor* executor)
    : context_(context) {
    node_ = std::make_shared<rclcpp::Node>("_autonomy_interface");

    executor->add_node(node_);
    status_subs_.reserve(kNumShells);
    for (int i = 0; i < kNumShells; i++) {
        status_subs_.emplace_back(node_->create_subscription<RobotStatusAdapter>(
            radio::topics::robot_status_topic(i), rclcpp::QoS(1),
            [this, i](const RobotStatus& status) { context_->robot_status.at(i) = status; }));
    }

    gameplay_debug_text_sub_ = node_->create_subscription<std_msgs::msg::String>(
        gameplay::topics::kDebugTextTopic, 1, [this](std_msgs::msg::String::SharedPtr message) {
            context_->behavior_tree = message->data;
        });
}

void AutonomyInterface::run() {}

}  // namespace ros2_temp
