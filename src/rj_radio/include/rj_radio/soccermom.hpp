#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/team_color.hpp>
#include <std_msgs/msg/string.hpp>

namespace tutorial {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    // Subscriber to listen to the team color
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;

    // Publisher to broadcast the fruit
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fruit_pub_;

    // Callback function that runs whenever a new color message arrives
    void color_callback(const rj_msgs::msg::TeamColor::SharedPtr msg);
};

}  // namespace tutorial