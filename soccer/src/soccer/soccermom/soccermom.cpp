#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

#include <rj_utils/logging.hpp>
#include "global_params.hpp"
#include "soccermom.hpp"

namespace tutorial {

SoccerMom::SoccerMom()
    : rclcpp::Node{"tutorial", rclcpp::NodeOptions{}
                        .automatically_declare_parameters_from_overrides(true)
                        .allow_undeclared_parameters(true)} { // if the subscriber doesn't work, rj_msgs/msg/TeamColor as the topic  // /referee/team_color
    //subscription = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&SoccerMom::topic_callback, this, std::placeholders::_1));
    subscription = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            if (color->is_blue) {
                teamColor = "blueberries";
            } else {
                teamColor = "banana";
            }
            //RCLCPP_INFO(this->get_logger(), teamColor);  
        });
    publisher = this->create_publisher<std_msgs::msg::String>("/team_fruit", 10);
    timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&SoccerMom::timer_callback, this));
    teamColor = "undecided";
}

void SoccerMom::timer_callback() {
    auto message = std_msgs::msg::String();
    //message.data = "test";
    message.data = teamColor;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher->publish(message);
}

void SoccerMom::topic_callback(const std_msgs::msg::String & msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    //teamColor = msg.data.c_str;
}

/*
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto soccermom = std::make_shared<tutorial::SoccerMom>();
    start_global_param_provider(soccermom.get(), kGlobalParamServerNode);
    rclcpp::spin(soccermom);
}*/

}