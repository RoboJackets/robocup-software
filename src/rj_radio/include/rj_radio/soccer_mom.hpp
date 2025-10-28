#pragma once

#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/radio/robot_status.hpp>
#include <rj_common/robot_intent.hpp>
#include <rj_common/strategy/positions.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/alive_robots.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/motion_setpoint.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_msgs/msg/fruit_type.hpp>
#include <rj_param_utils/param.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>


namespace radio {
    constexpr auto kRadioParamModule = "soccermom";
    class SoccerMom: public rclcpp::Node {
        public:
            SoccerMom();
        
        private:
            rclcpp::Publisher<rj_msgs::msg::FruitType>::SharedPtr fruit_pub_;
            rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
            // Ros param provider for initializing the radio node
            ::params::LocalROS2ParamProvider param_provider_;

    };
}