#include "soccer_mom_node.hpp"
#include <spdlog/spdlog.h>

#include <std_msgs/msg/string.hpp>

namespace tutorial {

SoccerMom::SoccerMom()
    : Node("soccer_mom") {
    team_fruit_pub = create_publisher<std_msgs::msg::String>(topics::kTeamFruitPub, rclcpp::QoS(1));
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorPub, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            auto msg = std_msgs::msg::String();
            if (color->is_blue) {
                 msg.data = "blueberries";
                // team_fruit_pub->publish("blueberries");
            } else {
                msg.data = "banana";
                // team_fruit_pub->publish("banana");
            }
            team_fruit_pub->publish(msg);
        });
}

}  // namespace tutorial