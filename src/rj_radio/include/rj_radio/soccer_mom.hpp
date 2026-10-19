#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/team_color.hpp>
#include <std_msgs/msg/string.hpp>

namespace tutorial {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    // Ros subscriber for the team's color.
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;

    // Ros publisher for the team's fruit.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    /**
     * @brief Wrapper over the local publisher to publish a team fruit
     */
    void publish_team_fruit(bool is_blue);
};

}  // namespace tutorial
