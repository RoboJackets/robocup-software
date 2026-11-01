#include "rj_radio/soccer_mom.hpp"

namespace tutorial {

SoccerMom::SoccerMom() : Node{"soccer_mom"} {
    // Publisher: transient_local so a late listener still gets the latest fruit.
    fruit_pub_ =
        create_publisher<std_msgs::msg::String>("/team_fruit", rclcpp::QoS(1).transient_local());

    // Subscriber: same QoS as the radio uses for the team color topic.
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            std_msgs::msg::String fruit;
            fruit.data = color->is_blue ? "blueberries" : "banana";
            fruit_pub_->publish(fruit);
        });
}

}  // namespace tutorial