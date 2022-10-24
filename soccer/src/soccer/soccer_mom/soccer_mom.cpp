#include "soccer_mom.hpp"

namespace tutorial {

SoccerMom::SoccerMom() : Node("soccer_mom") {
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorPub, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            pick_fruit(color->is_blue);
        });

    team_fruit_pub_ =
        create_publisher<std_msgs::msg::String>(tutorial::topics::kSoccerMomFruitPub, rclcpp::QoS(1));
}

void SoccerMom::pick_fruit(bool is_blue) {
    auto message = std_msgs::msg::String();
    message.data = (is_blue) ? "blueberries" : "banana";
    team_fruit_pub_->publish(message);
}

}  // namespace tutorial