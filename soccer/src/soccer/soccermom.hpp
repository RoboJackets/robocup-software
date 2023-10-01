#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/motion_setpoint.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_param_utils/param.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>

namespace tutorial {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    void send();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr subscription_;
    bool is_blue_;
};

}
