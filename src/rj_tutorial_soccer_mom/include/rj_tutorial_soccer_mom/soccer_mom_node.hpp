#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <std_msgs/msg/string.hpp>

namespace tutorial {

class SoccerMomNode : public rclcpp::Node {
public:
    SoccerMomNode();

private:
    void team_color_callback(rj_msgs::msg::TeamColor::SharedPtr msg);

    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fruit_pub_;
};

}  // namespace tutorial
