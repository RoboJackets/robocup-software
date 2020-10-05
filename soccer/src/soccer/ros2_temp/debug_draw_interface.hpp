#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_drawing_msgs/msg/debug_draw.hpp>

#include "context.hpp"

namespace ros2_temp {

class DebugDrawInterface : public Node {
public:
    DebugDrawInterface(Context* context, rclcpp::Executor* executor);

    void run() override;

private:
    rclcpp::Node::SharedPtr node_;
    Context* context_;
    rclcpp::Subscription<rj_drawing_msgs::msg::DebugDraw>::SharedPtr debug_draw_sub_;
    std::unordered_map<std::string, rj_drawing_msgs::msg::DebugDraw::SharedPtr> latest_;
};

}  // namespace ros2_temp
