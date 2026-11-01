#include "rj_radio/soccer_mom.hpp"

namespace tutorial {

SoccerMom::SoccerMom() : Node{"soccer_mom"} {

    team_fruit_pub_ =
        create_publisher<std_msgs::msg::String>(radio::topics::kTeamFruitTopic, rclcpp::QoS(1).transient_local());

    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {
            std_msgs::msg::String fruit;

            fruit.data = color->is_blue ? "blueberries" : "banana";

            team_fruit_pub_->publish(fruit);
        });
}

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // Start ROS

    auto soccer_mom = std::make_shared<tutorial::SoccerMom>();

    rclcpp::spin(soccer_mom);
}