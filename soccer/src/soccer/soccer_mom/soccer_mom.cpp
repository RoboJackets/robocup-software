#include <rclcpp/rclcpp.hpp>
#include "soccer_mom.hpp"
#include <string>
#include "std_msgs/msg/string.hpp"

namespace tutorial {
    SoccerMom::SoccerMom() : Node("soccer_mom") {
    _team_color_sub = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
        teamColorCallback(color->is_blue);
    });
    _team_fruit_pub = this->create_publisher<std_msgs::msg::String>("soccer_mom/team_fruit", 10);
    
    }

    void SoccerMom::teamColorCallback(bool isBlue) {
        if (isBlue) {
            tf.data = "blueberries";
        } else {
            tf.data = "banana";
        }
        this->_team_fruit_pub->publish(tf);
    }

} // namespace tutorial