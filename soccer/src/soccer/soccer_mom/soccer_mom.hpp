#pragma once

#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/motion_setpoint.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_param_utils/param.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>
#include <std_msgs/msg/string.hpp>

#include "robot_intent.hpp"
#include "strategy/coach/coach_node.hpp"

namespace soccer_mom {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr team_fruit_pub_;
};

}  // namespace soccer_mom
