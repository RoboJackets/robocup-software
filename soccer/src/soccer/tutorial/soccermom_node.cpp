#include "soccermom_node.hpp"

namespace tutorial {

SoccerMomNode::SoccerMomNode() : rclcpp::Node("soccermom") {
    auto curr_color = NULL;
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorPub, rclcpp::QoS(1).transient_local(),
        [&](rj_msgs::msg::TeamColor::SharedPtr color) { curr_color = color->is_blue; });
    publisher_ = create_publisher<std_msgs::msg::String>("/team_fruit", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(1000), [&]() {
        auto message = std_msgs::msg::String();
        if (curr_color) {
            message.data = "Team Color: Blue | Fruit: Blueberries";
        } else {
            message.data = "Team Color: Yellow | Fruit: Banana";
        }
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    });
}

}  // namespace tutorial
