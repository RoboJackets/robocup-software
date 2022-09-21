#include "SoccerMom.hpp"

namespace tutorial {

SoccerMom::SoccerMom() : rclcpp::Node("SoccerMom") {
    publisher_ = create_publisher<std_msgs::msg::String>("/team_fruit", rclcpp::QoS(1));

    auto team_color = NULL;
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorPub, rclcpp::QoS(1),
        [&](rj_msgs::msg::TeamColor::SharedPtr color) { team_color = color->is_blue; });

    tick_timer_ = create_wall_timer(std::chrono::milliseconds(500), [&]() {
        auto message = std_msgs::msg::String();
        if (team_color) {
            message.data = "blueberries";
        } else {
            message.data = "banana";
        }
        RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    });
}

}  // namespace tutorial
