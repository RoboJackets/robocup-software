#pragma once

#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_param_utils/param.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>

#include <std_msgs/msg/string.hpp>

namespace tutorial {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

protected:
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr team_fruit_pub;
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
};

}  // namespace tutorial
