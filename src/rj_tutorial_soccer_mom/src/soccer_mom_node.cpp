#include "rj_tutorial_soccer_mom/soccer_mom_node.hpp"
#include <rj_utils/logging.hpp>

namespace tutorial {

SoccerMomNode::SoccerMomNode() : rclcpp::Node("soccer_mom") {
    fruit_pub_ = create_publisher<std_msgs::msg::String>("team_fruit", 10);

    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr msg) { team_color_callback(msg); });
}

void SoccerMomNode::team_color_callback(rj_msgs::msg::TeamColor::SharedPtr msg) {
    std_msgs::msg::String fruit_msg;
    if (msg->is_blue) {
        fruit_msg.data = "blueberries";
    } else {
        fruit_msg.data = "banana";
    }
    fruit_pub_->publish(fruit_msg);
    RCLCPP_INFO(get_logger(), "Team color %s -> publishing %s", msg->is_blue ? "blue" : "yellow", fruit_msg.data.c_str());
}

}