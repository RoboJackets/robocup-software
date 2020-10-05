#pragma once

#include <rclcpp/rclcpp.hpp>
#include "context.hpp"

namespace ros2_temp {

class DebugDrawerInterface {
public:
    DebugDrawerInterface(Context* context, rclcpp::Executor* executor);

private:
    DebugDrawer* debug_drawer_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<
};

}  // namespace ros2_temp