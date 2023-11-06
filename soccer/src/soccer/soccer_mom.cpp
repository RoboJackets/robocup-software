#include "soccer_mom.hpp"

namespace tutorial {

SoccerMom::SoccerMom() : Node{"soccer_mom"} {
    team_color_pub_ =
        create_publisher<std_msgs::msg::String>("/team_fruit", rclcpp::QoS(1).transient_local());
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) { this->publish(color->is_blue); });
}

void SoccerMom::publish(bool isBlue) {
    auto message = std_msgs::msg::String();
    if (isBlue) {
        message.data = "blueberries";
    } else {
        message.data = "bananas";
    }
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    team_color_pub_->publish(message);
}
}  // namespace tutorial