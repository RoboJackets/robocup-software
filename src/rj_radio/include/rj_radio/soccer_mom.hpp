#pragma once

#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <rj_msgs/msg/team_color.hpp>
#include "std_msgs/msg/string.hpp"
#include <rj_param_utils/param.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>
#include <rj_utils/logging.hpp>
namespace tutorial {

    class Soccer_Mom : public rclcpp::Node {
        public:
        Soccer_Mom();

        private:
        bool is_blue = false;
        void publisher_callback();
        void subscriber_callback(rj_msgs::msg::TeamColor::SharedPtr) ;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;


    };
}

