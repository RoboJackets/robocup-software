#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/team_color.hpp>

#include "std_msgs/msg/string.hpp"

namespace tutorial {

/**
 * This node is part of the tutorial project. It's purpose is to get members
 * acquainted with ROS2 and C++. The task of this node is to subscribe to the
 * current team color, and publish a string to a new topic.
 */

class SoccerMomNode : public rclcpp::Node {
public:
    SoccerMomNode();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

}  // namespace tutorial
