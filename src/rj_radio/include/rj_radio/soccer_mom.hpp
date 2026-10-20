#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <std_msgs/msg/string.hpp>
#include <rj_utils/logging.hpp>

namespace tutorial {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fruit_pub_;
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
};

} // namespace tutorial