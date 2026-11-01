#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/team_color.hpp>

namespace tutorial {

/**
 * SoccerMom listens to our team color and publishes a matching fruit
 * on /team_fruit: "banana" for yellow, "blueberries" for blue.
 */
class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr team_fruit_pub_;
};

}  // namespace tutorial