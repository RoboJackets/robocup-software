#include "autonomy_interface.hpp"

#include <spdlog/spdlog.h>

#include <context.hpp>
#include <rj_constants/topic_names.hpp>

#include "radio/packet_convert.hpp"

namespace ros2_temp {

AutonomyInterface::AutonomyInterface(Context* context, rclcpp::Executor* executor)
    : context_(context) {
    node_ = std::make_shared<rclcpp::Node>("_autonomy_interface");

    executor->add_node(node_);
    intent_pubs_.reserve(kNumShells);
    status_subs_.reserve(kNumShells);
    for (int i = 0; i < kNumShells; i++) {
        intent_pubs_.emplace_back(node_->create_publisher<RobotIntent::Msg>(
            gameplay::topics::robot_intent_pub(i) + "_old_soccer", rclcpp::QoS(1).transient_local()));
        status_subs_.emplace_back(node_->create_subscription<rj_msgs::msg::RobotStatus>(
            radio::topics::robot_status_pub(i), rclcpp::QoS(1),
            [this, i](rj_msgs::msg::RobotStatus::SharedPtr status) {  // NOLINT
                ConvertRx::ros_to_status(*status, &context_->robot_status.at(i));
            }));
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

}  // namespace ros2_temp
