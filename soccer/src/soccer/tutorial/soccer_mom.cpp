#include "soccer_mom.hpp"

#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/team_color.hpp>

#include "std_msgs/msg/string.hpp"

namespace tutorial {

SoccerMom::SoccerMom() : Node("soccer_mom") {
    this->publisher_ = this->create_publisher<std_msgs::msg::String>("/team_fruit", 10);
    this->subscriber_ = this->create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            auto message = std_msgs::msg::String();
            message.data = "blueberries";
            if (!color->is_blue) {
                message.data = "banana";
            }

            this->publisher_->publish(message);
        });
}
}  // namespace tutorial
