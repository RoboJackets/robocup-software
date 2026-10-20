// includes and header stuff
#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/team_color.hpp>
#include <std_msgs/msg/string.hpp>

namespace tutorial {
class SoccerMom : public rclcpp::Node {  // inherits rcl
    // publics
public:
    SoccerMom();

private:
    // subscriber === READ team color
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;

    // publisher === announce fruit
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fruit_pub_;

};
}  // namespace tutorial