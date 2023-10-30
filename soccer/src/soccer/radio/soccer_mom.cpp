// use tutorial name space /referee/team_color publish to /team_fruit

#include "soccer_mom.hpp"
#include <spdlog/spdlog.h>


namespace tutorial {
    SoccerMom::SoccerMom() : rclcpp::Node{"soccer_mom"} {
                team_fruit_pub_ = create_publisher<std_msgs::msg::String>(
                    "team_fruit", rclcpp::QoS(1));

                team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
                    referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
                    [this](rj_msgs::msg::TeamColor::SharedPtr color) { 
                        std_msgs::msg::String message;
                        if (color->is_blue) {
                            message.data = "blueberries";
                        } else {
                            message.data = "bananas";
                        }
                        team_fruit_pub_->publish(message);
                    });
            

        
    }
}