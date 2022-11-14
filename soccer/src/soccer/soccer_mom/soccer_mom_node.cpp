#include "soccer_mom_node.hpp"

#include <cmath>
#include <stdexcept>

#include <spdlog/spdlog.h>

#include <rj_common/network.hpp>
#include <rj_protos/ssl_simulation_control.pb.h>
#include <rj_protos/ssl_simulation_robot_control.pb.h>
#include <rj_protos/ssl_simulation_robot_feedback.pb.h>


#include <std_msgs/msg/string.hpp>


using namespace std;

namespace tutorial {

SoccerMom::SoccerMom() 

    : Node("Soccer_Mom")
    {
    robot_fruit_pubs_ = create_publisher<std_msgs::msg::String>("team_fruit", 10);
    
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorPub, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) { 
            
                //If color is blue, publishes blueberries, else publishes bananas
                auto message = std_msgs::msg::String();
                    if (color->is_blue == true) {
                        message.data = "Blueberries";
                        robot_fruit_pubs_->publish(message);
                    } else {
                        message.data = "Bananas";
                        robot_fruit_pubs_->publish(message);

    };
        });
    }


}


