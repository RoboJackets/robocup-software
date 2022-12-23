#pragma once

#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/team_color.hpp>
#include <rj_msgs/msg/team_fruit.hpp>

namespace tutorial {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    //Used to publish team_fruit string to TeamFruit based on color subscription to TeamColor
    void publish(const rj_msgs::msg::TeamColor::SharedPtr color);

    //Subscription to TeamColor
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;

    //Publisher for TeamFruit
    rclcpp::Publisher<rj_msgs::msg::TeamFruit>::SharedPtr team_fruit_pub_;
};

}
