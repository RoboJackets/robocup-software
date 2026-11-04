#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/team_color.hpp>
#include <std_msgs/msg/string.hpp>

namespace tutorial {

/**
 * Subscribes to the team color and publishes a matching fruit to /team_fruit.
 */
class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr color_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fruit_pub_;
};

}  // namespace tutorial