#include "soccer_mom.hpp"
#include <spdlog/spdlog.h>

namespace soccer_mom{

SoccerMom::SoccerMom()
    : Node{"soccer_mom", rclcpp::NodeOptions{}
                        .automatically_declare_parameters_from_overrides(true)
                        .allow_undeclared_parameters(true)}
    {
    team_fruit_pub_ = create_publisher<std_msgs::msg::String>("/team_fruit", 10);
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
	    auto message = std_msgs::msg::String();
            message.data = "blueberries";
            if (!color->is_blue) {
                message.data = "banana";
            }
		team_fruit_pub_->publish(message);
        });
}

}
