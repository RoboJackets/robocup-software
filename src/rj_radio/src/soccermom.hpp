#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rj_msgs/msg/team_color.hpp>

namespace tutorial {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fruit_pub_;

    void color_callback(const rj_msgs::msg::TeamColor::SharedPtr msg);
};

}  // namespace tutorial
