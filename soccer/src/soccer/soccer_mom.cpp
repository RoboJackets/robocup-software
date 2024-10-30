#include "soccer_mom.hpp"

SoccerMom::SoccerMom() : rclcpp::Node{"SoccerMom"} {
    soccer_mom_pub_ = create_publisher<std_msgs::msg::String>("team_fruit", rclcpp::QoS(1));

    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {
            auto message = std_msgs::msg::String();
            if (color->is_blue) {
                message.data = "blueberries";
            } else {
                message.data = "bananas";
            }
            soccer_mom_pub_->publish(message);
        });
}
