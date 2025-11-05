#include "rj_radio/soccer_mom.hpp"

namespace tutorial {

SoccerMom::SoccerMom() : Node("soccer_mom") {
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr msg) { team_color_callback(msg); });

    team_fruit_pub_ = create_publisher<std_msgs::msg::String>("/team_fruit", rclcpp::QoS(1));
}

void SoccerMom::team_color_callback(const rj_msgs::msg::TeamColor::SharedPtr msg) {
    std_msgs::msg::String fruit_msg;

    if (msg->is_blue) {
        fruit_msg.data = "blueberries";
    } else {
        fruit_msg.data = "banana";
    }

    team_fruit_pub_->publish(fruit_msg);
}

}  // namespace tutorial

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tutorial::SoccerMom>());
    rclcpp::shutdown();
    return 0;
}
