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
    status_subs_.reserve(kNumShells);
    for (int i = 0; i < kNumShells; i++) {
        status_subs_.emplace_back(node_->create_subscription<rj_msgs::msg::RobotStatus>(
            radio::topics::robot_status_pub(i), rclcpp::QoS(1),
            [this, i](rj_msgs::msg::RobotStatus::SharedPtr status) {  // NOLINT
                ConvertRx::ros_to_status(*status, &context_->robot_status.at(i));
            }));
    }

    gameplay_debug_text_sub_ = node_->create_subscription<std_msgs::msg::String>(
        gameplay::topics::kDebugTextPub, 1, [this](std_msgs::msg::String::SharedPtr message) {
            context_->behavior_tree = message->data;
        });
}

void AutonomyInterface::run() {}

}  // namespace ros2_temp
