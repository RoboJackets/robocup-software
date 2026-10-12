// includes and header stuff
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace tutorial {
    class SoccerMom: public rclcpp::Node { // inherits rcl
        // publics
        public: 
            SoccerMom();

        private:
            // subscriber === READ team color
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr team_color_sub_;

            // publisher === announce fruit
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fruit_pub_;

            // callback method
            void color_callback(const std_msgs::msg::String::SharedPtr msg);
    };
}