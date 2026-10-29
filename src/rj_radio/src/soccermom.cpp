#include "rj_radio/soccermom.hpp"
#include "rj_constants/topic_names.hpp"

namespace tutorial {

SoccerMom::SoccerMom()
    : Node{"soccer_mom", rclcpp::NodeOptions{}
                             .automatically_declare_parameters_from_overrides(true)
                             .allow_undeclared_parameters(true)} {
    team_fruit_pub_ =
        create_publisher<std_msgs::msg::String>("/team_fruit", rclcpp::QoS(1));

    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {
            std_msgs::msg::String fruit_msg;
            fruit_msg.data = color->is_blue ? "blueberries" : "banana";
            team_fruit_pub_->publish(fruit_msg);
        });
}
}