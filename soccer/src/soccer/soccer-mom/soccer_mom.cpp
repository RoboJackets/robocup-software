#include "soccer_mom.hpp"

namespace tutorial {
SoccerMom::SoccerMom() : Node("soccermom") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/team_fruit", rclcpp::QoS(1));
    team_color_sub_ = this->create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorPub, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            auto message = std_msgs::msg::String();
            if (color->is_blue) {
                message.data = "blueberries";
                this->publisher_->publish(message);
            } else {
                message.data = "banana";
                this->publisher_->publish(message);
            }
        });
}
}  // namespace tutorial