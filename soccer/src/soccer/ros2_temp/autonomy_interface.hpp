#pragma once

#include <rclcpp/rclcpp.hpp>
#include "robot_intent.hpp"

namespace ros2_temp {

class AutonomyInterface {
public:
    AutonomyInterface(Context* context, rclcpp::Executor* executor);
private:
    std::shared_ptr<rclcpp::Node> node_;
    Context* context_;
    std::vector<rclcpp::Publisher<RobotIntent::Msg>::SharedPtr> intent_pubs_;
};

} // namespace ros2_temp