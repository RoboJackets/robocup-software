#include "soccer_mom.hpp"

namespace soccer_mom {
    SoccerMom::SoccerMom() {
        team_color_pub_ = this->create_publisher<std_msgs::msg::String>("/team_fruit", 10);
        tick_timer_ = this->create_wall_timer(
        500ms, std::bind(&SoccerMom::timer_callback, this));
        team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
            referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
            [this](rj_msgs::msg::TeamColor::SharedPtr color) {
                blue_team_ = color->is_blue;
            });
    }

    void SoccerMom::timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = blue_team_ ? "blueberries" : "banana";
        message.data += std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        team_color_pub_->publish(message);
    }
}
