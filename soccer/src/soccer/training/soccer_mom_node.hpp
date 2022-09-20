#pragma once

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

#include <rj_msgs/msg/team_color.hpp>
#include <memory>
#include <string>

using TeamColorMsg = rj_msgs::msg::TeamColor;

class SoccerMom: public rclcpp::Node {
public:
    SoccerMom() : rclcpp::Node("soccer_mom") {
        team_color_subscription_ = this->create_subscription<TeamColorMsg>(
            "/referee/team_color", rclcpp::QoS(1), std::bind(&SoccerMom::team_color_callback, this, std::placeholders::_1));

        team_fruit_publisher_ = this->create_publisher<std_msgs::msg::String>("/team_fruit", rclcpp::QoS(1));
    }
private:
    void team_color_callback(const TeamColorMsg::SharedPtr team_color) const {
        auto message = std_msgs::msg::String();
        if (team_color->is_blue) {
            message.data = "blueberries";
            team_fruit_publisher_->publish(message);
        } else {
            message.data = "banana";
            team_fruit_publisher_->publish(message);
        }
    }

    rclcpp::Subscription<TeamColorMsg>::SharedPtr team_color_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr team_fruit_publisher_;
};