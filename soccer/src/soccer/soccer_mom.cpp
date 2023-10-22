#include <soccer_mom.hpp>

namespace tutorial{
Soccer_Mom::Soccer_Mom()
    : Node("soccer_mom")
    {
        publisher_ = create_publisher<std_msgs::msg::String>("team_fruit", 10);
        subscription_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            // RCLCPP_INFO(this->get_logger(), "ASDF %s", color->is_blue);
            recieve(color->is_blue);
        });
        timer_ = create_wall_timer(std::chrono::milliseconds(16), [this]() { send(); });
        // is_blue_ = true;
}

void Soccer_Mom::recieve(bool b) {
    // if (b == is_blue_) {
    //     return;
    // }
    is_blue_ = b;
}

void Soccer_Mom::send() {
    // RCLCPP_INFO(this->get_logger(), "Team: '%s'", is_blue_);

    auto msg = std_msgs::msg::String();
    if (is_blue_) {
        msg.data = "blueberries";
    }
    else {
        msg.data = "bananas";
    }
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher_->publish(msg);
}
}