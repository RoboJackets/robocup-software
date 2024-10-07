#include "soccermom.hpp"

#include <spdlog/spdlog.h>

Soccermom::Soccermom(bool blue_team)
    : Node{"soccermom", rclcpp::NodeOptions{}
                            .automatically_declare_parameters_from_overrides(true)
                            .allow_undeclared_parameters(true)}

{
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            team_fruit_pub_ =
                create_publisher<std_msgs::msg::String>("/team_fruit", rclcpp::QoS(10));

            if (color->is_blue) {
                // publish "blueberries"
                auto message = std_msgs::msg::String();
                message.data = "blueberries";
                team_fruit_pub_->publish(message);
            } else {
                // publish "bananas"
                auto message = std_msgs::msg::String();
                message.data = "bananas";
                team_fruit_pub_->publish(message);
            }

        });

    team_fruit_pub_ = create_publisher<std_msgs::msg::String>("/team_fruit", rclcpp::QoS(10));

    if (blue_team) {
        // publish "blueberries"
        auto message = std_msgs::msg::String();
        message.data = "blueberries";
        team_fruit_pub_->publish(message);
    } else {
        // publish "bananas"
        auto message = std_msgs::msg::String();
        message.data = "bananas";
        team_fruit_pub_->publish(message);
    }
}
