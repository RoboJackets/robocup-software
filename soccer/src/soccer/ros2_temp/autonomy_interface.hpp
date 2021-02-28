#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/robot_status.hpp>

#include "node.hpp"
#include "robot_intent.hpp"

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
    std::shared_ptr<rclcpp::Node> node_;
    Context* context_;
    std::vector<rclcpp::Publisher<RobotIntent::Msg>::SharedPtr> intent_pubs_;
    std::vector<rclcpp::Subscription<rj_msgs::msg::RobotStatus>::SharedPtr> status_subs_;
};

}  // namespace ros2_temp