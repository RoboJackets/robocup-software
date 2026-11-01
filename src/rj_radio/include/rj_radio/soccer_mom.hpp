#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <std_msgs/msg/string.hpp>

namespace tutorial {

/**
 * @brief Picks a fruit to match our team color.
 *
 * @details Subscribes to the team color topic and publishes a String to /team_fruit:
 *  - yellow team -> "banana"
 *  - blue team   -> "blueberries"
 */
class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    // Ros subscriber for the team's color.
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;

    // Ros publisher for the chosen fruit.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fruit_pub_;
};

}  // namespace tutorial