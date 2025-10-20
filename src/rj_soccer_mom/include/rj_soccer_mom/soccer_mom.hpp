#include <chrono>
#include <memory>
#include <string>

#include <rj_msgs/msg/team_color.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr subscription_;
    bool is_blue = false;
};