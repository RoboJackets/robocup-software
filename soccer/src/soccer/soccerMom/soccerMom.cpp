#include <rclcpp/rclcpp.hpp>
#include "soccerMom.hpp"
#include <string>
#include "std_msgs/msg/string.hpp"


namespace tutorial {
    SoccerMom::SoccerMom() : Node("soccer_mom") {
        team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            teamColorCallBack(color->is_blue);
        });

        team_fruit_publisher = create_publisher<std_msgs::msg::String>("soccer_mom/team_fruit", 10);
    }

    

    void SoccerMom::teamColorCallBack(bool isBlue) {
        if (isBlue) {
            teamFruit.data = "blueberries";
        } else {
            teamFruit.data = "banana";
        }
        this->team_fruit_publisher->publish(teamFruit);
    }
}