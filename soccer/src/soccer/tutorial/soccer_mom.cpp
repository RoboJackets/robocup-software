#include "soccer_mom.hpp"
#include <rj_msgs/msg/team_fruit.hpp>

#include <rclcpp/rclcpp.hpp>

#include <spdlog/spdlog.h>

namespace tutorial {

SoccerMom::SoccerMom() : Node("soccer_mom") {
    team_color_sub_ = this->create_subscription<rj_msgs::msg::TeamColor>(
        "/referee/team_color", 
        rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            publish(color);
        });

    team_fruit_pub_ = this->create_publisher<rj_msgs::msg::TeamFruit>("/team_fruit", 10);
}

void SoccerMom::publish(const rj_msgs::msg::TeamColor::SharedPtr color) {
    auto message = rj_msgs::msg::TeamFruit();
    message.team_fruit = (color -> is_blue) ? "blueberries" : "banana";
    team_fruit_pub_->publish(message); 
}

}
