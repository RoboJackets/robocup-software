#include <rclcpp/rclcpp.hpp>
#include "soccer_mom.hpp"


namespace tutorial {
SoccerMom::SoccerMom() : Node("soccer_mom") {
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorPub, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            auto message = std_msgs::msg::String();
            if (color->is_blue) {
                message.data = "blueberries";
            } else {
                message.data = "banana";
            }
            publisher_->publish(message);
        });
    publisher_ = create_publisher<std_msgs::msg::String>("team_fruit", 10);
}
}