#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rj_msgs/msg/team_color.hpp>

using namespace std::chrono_literals;


namespace Tutorial {
    class SoccerMom : public rclcpp::Node {
    public:
        SoccerMom();

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fruitPublisher_;
        rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr colorSubscriber_;
        rclcpp::TimerBase::SharedPtr timer_;

        bool isBlue;

        void timerCallback();

        void subscriberCallback(const rj_msgs::msg::TeamColor::SharedPtr msg);

    };
}