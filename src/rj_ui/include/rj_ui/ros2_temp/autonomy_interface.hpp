#pragma once

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/node.hpp>
#include <rj_common/radio/robot_status.hpp>
#include <rj_common/robot_intent.hpp>
#include <rj_common/topic_names.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <std_msgs/msg/string.hpp>

#include "rj_ui/context.hpp"

namespace ros2_temp {

class AutonomyInterface : public Node {
public:
    AutonomyInterface(Context* context, rclcpp::Executor* executor);
    ~AutonomyInterface() override = default;

    void run() override;

    AutonomyInterface(const AutonomyInterface&) = delete;
    AutonomyInterface& operator=(const AutonomyInterface&) = delete;
    AutonomyInterface(AutonomyInterface&&) = delete;
    AutonomyInterface& operator=(AutonomyInterface&&) = delete;

private:
    using RobotStatusAdapter = rclcpp::TypeAdapter<RobotStatus, rj_msgs::msg::RobotStatus>;

    std::shared_ptr<rclcpp::Node> node_;
    Context* context_;
    std::vector<rclcpp::Subscription<RobotStatusAdapter>::SharedPtr> status_subs_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gameplay_debug_text_sub_;
};

}  // namespace ros2_temp