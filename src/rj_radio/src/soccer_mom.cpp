#include "rj_radio/soccer_mom.hpp"

namespace tutorial {

SoccerMom::SoccerMom() : Node("soccer_mom") {
    // Publisher for the fruit choice 
    team_fruit_pub_ = create_publisher<std_msgs::msg::String>("/team_fruit", 10);

    // Subscribe to team color, using the same topic constant and QoS as radio.cpp
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            // Pick the fruit that matches  team color
            auto fruit = std_msgs::msg::String();
            fruit.data = color->is_blue ? "blueberries" : "banana";

            team_fruit_pub_->publish(fruit);
        });

    RCLCPP_INFO(get_logger(), "SoccerMom has been started.");
}

}  // namespace tutorial

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tutorial::SoccerMom>());
    rclcpp::shutdown();
    return 0;
}