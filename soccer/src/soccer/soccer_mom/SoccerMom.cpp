#include "SoccerMom.hpp"

#include <spdlog/spdlog.h>

namespace tutorial {
    SoccerMom::SoccerMom()
        : Node{"SoccerMom", rclcpp::NodeOptions{}
                        .automatically_declare_parameters_from_overrides(true)
                        .allow_undeclared_paramteres(true)} {

        team_fruit_pub_ = create_publisher<std_msgs::msg::String>("team_fruit", 10);
        team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
            referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
            [this](rj_msgs::msg::TeamColor::SharedPtr color) { 
                auto message = std_msgs::msg::String();
                message.data = "blueberries";
                if (!color->is_blue) {
                    message.data = "banana";
                }
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()));
                team_fruit_pub_->publish(message);
        });
        
        }
}
