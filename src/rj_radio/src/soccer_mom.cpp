#include "rj_radio/soccer_mom.hpp"

#include <rj_constants/topic_names.hpp>

namespace tutorial {

SoccerMom::SoccerMom() : Node("soccer_mom") {
    fruit_pub_ =
        create_publisher<std_msgs::msg::String>("/team_fruit", rclcpp::QoS(1).transient_local());

    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {
            std_msgs::msg::String fruit;
            fruit.data = color->is_blue ? "blueberries" : "banana";
            fruit_pub_->publish(fruit);
        });
}

}  // namespace tutorial

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tutorial::SoccerMom>());
    rclcpp::shutdown();
    return 0;
}
