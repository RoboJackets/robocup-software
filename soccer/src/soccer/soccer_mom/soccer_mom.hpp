#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rj_constants/topic_names.hpp>

#include "std_msgs/msg/string.hpp"
#include "rj_msgs/msg/team_color.hpp"

namespace tutorial {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    void publish_fruit(bool is_blue_team);
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mom_msg_pub_;
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    const std::string pub_topic = "team_fruit";
};

} // namespace tutorial