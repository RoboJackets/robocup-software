#include "autonomy_interface.hpp"

#include <rj_constants/topic_names.hpp>

namespace ros2_temp {

AutonomyInterface::AutonomyInterface(Context* context, rclcpp::Executor* executor)
    : context_(context) {
    node_ = std::make_shared<rclcpp::Node>("_autonomy_interface");

    executor->add_node(node_);
    intent_pubs_.reserve(kNumShells);
    for (int i = 0; i < kNumShells; i++) {
        intent_pubs_.emplace_back(node_->create_publisher<RobotIntent::Msg>(
            gameplay::topics::robot_intent_pub(i),
            rclcpp::QoS(1).transient_local()));
    }
}

} // namespace ros2_temp