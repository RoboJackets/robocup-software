#include "soccer_mom.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rj_msgs/msg/team_fruit.hpp>

#include <spdlog/spdlog.h>

//namespace soccer_mom {

//DEFINE_FLOAT64(kRadioParamModule, timeout, 0.25,
//               "Timeout after which radio will assume a robot is disconnected. Seconds.");

SoccerMom::SoccerMom()
    : Node("soccer_mom")
    {
    team_color_sub_ = this->create_subscription<rj_msgs::msg::TeamColor>(
                         "/referee/team_color", 
                         rclcpp::QoS(1).transient_local(), 
                         [this](const rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
                                  publish(color);
                                  });
    
    team_fruit_pub_ = this->create_publisher<rj_msgs::msg::TeamFruit>(
                         "/team_fruit", 
                         rclcpp::QoS(1));
    }

//}

void SoccerMom::publish(const rj_msgs::msg::TeamColor::SharedPtr color) {
    auto message = rj_msgs::msg::TeamFruit();
    message.team_fruit = (color->is_blue) ? "blueberry": "banana";
    team_fruit_pub_->publish(message);

}

