#pragma once

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace tutorial {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    void poll_receive();
    void start_receive();
    void handle_receive(const std::string& data);
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    size_t count_;
};
}  // namespace tutorial