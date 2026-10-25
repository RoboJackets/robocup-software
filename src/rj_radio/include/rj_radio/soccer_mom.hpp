#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/team_color.hpp>
#include <std_msgs/msg/string.hpp>

namespace tutorial {

// SoccerMom is a small ROS 2 node.
// A node is one program in the stack. Nodes talk to each other through topics.
//
// This node listens for our team color and publishes a matching fruit:
//   blue   -> "blueberries"
//   yellow -> "banana"
//
// It is a subclass of rclcpp::Node, so it can create subscribers and publishers.
class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    // ROS calls this every time a new team-color message arrives.
    void on_team_color(const rj_msgs::msg::TeamColor::SharedPtr& color);

    // Listens to "referee/team_color". The message says whether we are blue.
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;

    // Publishes a std_msgs/String on "/team_fruit".
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fruit_pub_;
};

}  // namespace tutorial
