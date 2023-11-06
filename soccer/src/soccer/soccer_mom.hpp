#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <std_msgs/msg/string.hpp>

namespace tutorial {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

protected:
    void publish(bool isBlue);

private:
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr team_color_pub_;
};
}  // namespace tutorial