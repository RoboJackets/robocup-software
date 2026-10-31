#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/team_color.hpp>
#include <std_msgs/msg/string.hpp>

namespace tutorial {

/**
 * @brief Publishes a fruit on /team_fruit that matches our team color.
 */
class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fruit_pub_;
};

}  // namespace tutorial
