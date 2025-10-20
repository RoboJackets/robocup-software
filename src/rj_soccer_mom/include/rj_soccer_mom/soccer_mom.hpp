#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <rj_msgs/msg/team_color.hpp>

using namespace std::chrono_literals;

class SoccerMom : public rclcpp::Node
{
public:
  SoccerMom();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr subscription_;
  bool is_blue = false;
};