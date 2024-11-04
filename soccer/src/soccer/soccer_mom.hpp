#pragma once //so that we don't have duplicate identifiers

#include <rclcpp/rclcpp.hpp>
#include <rj_constants/topic_names.hpp>

#include "std_msgs/msg/string.hpp"
#include "rj_msgs/msg/team_color.hpp"

class SoccerMom : public rclcpp::Node { //extends the ROS node
public:
  SoccerMom();
private:
  rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_; // subscriber
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr soccer_mom_pub_; //publisher
  
};
