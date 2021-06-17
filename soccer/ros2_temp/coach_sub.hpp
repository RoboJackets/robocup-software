#pragma once

#include <rclcpp/executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/msg/position_assignment.hpp>

#include "context.hpp"
#include "node.hpp"

namespace ros2_temp {

class CoachSub : public Node {
public:
    CoachSub(Context* context, rclcpp::Executor* executor);

    void run() override {}

private:
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Subscription<rj_msgs::msg::PositionAssignment>::SharedPtr positions_sub_;

    std::thread worker_;

    Context* context_;
};

}  // namespace ros2_temp
