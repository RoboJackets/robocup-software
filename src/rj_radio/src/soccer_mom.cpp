#include "rj_radio/soccer_mom.hpp"

namespace tutorial {

SoccerMom::SoccerMom() : Node("soccer_mom") {
    // transient_local keeps the last fruit around, so `ros2 topic echo /team_fruit`
    // started later still sees it.
    fruit_pub_ =
        create_publisher<std_msgs::msg::String>("/team_fruit", rclcpp::QoS(1).transient_local());

    // Same QoS as the Radio node uses for this topic
    color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        "/referee/team_color", rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr msg) {
            std_msgs::msg::String fruit;
            fruit.data = msg->is_blue ? "blueberries" : "banana";
            fruit_pub_->publish(fruit);
        });
}

}  // namespace tutorial