#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_param_utils/param.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>

#include "std_msgs/msg/string.hpp"

namespace soccer_mom {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    void callback(const rj_msgs::msg::TeamColor::SharedPtr& color);
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    rclcpp::TimerBase::SharedPtr tick_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr team_color_pub_;
    bool blue_team_;
};

}  