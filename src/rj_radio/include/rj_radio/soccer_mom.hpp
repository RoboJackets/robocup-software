#pragma once

#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/team_color.hpp>
#include <std_msgs/msg/string.hpp>

namespace tutorial {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    // initializes published, subscribed, team_color, and the timer variable
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr subscription_;
    std::string team_color_;
    void timer_callback();
    void topic_callback(const rj_msgs::msg::TeamColor::SharedPtr msg);
};
}  // namespace tutorial