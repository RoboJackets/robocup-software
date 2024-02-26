#include "soccer_mom.hpp"

namespace tutorial {

SoccerMom::SoccerMom()
    : Node{"soccer_mom", rclcpp::NodeOptions{}
                             .automatically_declare_parameters_from_overrides(true)
                             .allow_undeclared_parameters(true)} {
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            publish_fruit(color->is_blue);
        });

    mom_msg_pub_ = create_publisher<std_msgs::msg::String>(pub_topic_, rclcpp::QoS(1));
}

void SoccerMom::publish_fruit(bool is_blue_team) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    if (is_blue_team) {
        msg->data = "blueberries";
    } else {
        msg->data = "banana";
    }
    mom_msg_pub_->publish(std::move(msg));
}

}  // namespace tutorial