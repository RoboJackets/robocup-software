#include "SoccerMom.hpp"

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_msgs/srv/quick_commands.hpp>
#include <rj_msgs/srv/quick_restart.hpp>

#include "referee/referee_base.hpp"

namespace tutorial {
SoccerMom::SoccerMom() : Node("soccer_mom") {
    _team_color_sub = this->create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            teamColorCallback(color->is_blue);
        });
    _team_fruit_pub = this->create_publisher<std_msgs::msg::String>("tutorial/team_fruit", 10);
}

void SoccerMom::teamColorCallback(bool isBlue) {
    if (isBlue) {
        tf.data = "Blueberries";
    } else {
        tf.data = "Bananas";
    }
    this->_team_fruit_pub->publish(tf);
}

}  // namespace tutorial