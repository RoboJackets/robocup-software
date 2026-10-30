#pragma once

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/team_color.hpp>

#include <std_msgs/msg/string.hpp>

namespace tutorial {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr team_fruit_pub_;

    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    // Whether or not the current team color is blue
    bool blue_team_;

   
};

}
