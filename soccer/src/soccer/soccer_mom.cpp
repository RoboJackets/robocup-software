#include "soccer_mom.hpp"

#include <chrono>

#include <spdlog/spdlog.h>

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace soccer_mom {
SoccerMom::SoccerMom() : Node{"soccer_mom"} {
    team_color_pub_ = create_publisher<std_msgs::msg::String>("/team_fruit", 10);
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            callback(color);
        });
}

void SoccerMom::callback(const rj_msgs::msg::TeamColor::SharedPtr& color) {
    blue_team_ = color->is_blue;
    auto message = std_msgs::msg::String();
    message.data = blue_team_ ? "blueberries" : "banana";
    SPDLOG_INFO("Publishing: {}", message.data);
    team_color_pub_->publish(message);
}
}  // namespace soccer_mom