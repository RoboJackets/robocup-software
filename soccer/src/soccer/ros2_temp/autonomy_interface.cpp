#include "autonomy_interface.hpp"

#include <spdlog/spdlog.h>

#include <context.hpp>
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

void AutonomyInterface::run() {
    for (int i = 0; i < kNumShells; i++) {
        const auto& intent = context_->robot_intents.at(i);
        if (intent.is_active) {
            intent_pubs_.at(i)->publish(rj_convert::convert_to_ros(intent));
        }
    }
}

} // namespace ros2_temp