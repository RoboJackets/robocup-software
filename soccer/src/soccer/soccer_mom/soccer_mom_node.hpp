#pragma once

#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
// #include <rj_msgs/msg/manipulator_setpoint.hpp>
// #include <rj_msgs/msg/motion_setpoint.hpp>
// #include <rj_msgs/msg/team_color.hpp>
#include <rj_param_utils/param.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>

#include <std_msgs/msg/string.hpp>

#include <rj_msgs/msg/team_color.hpp>



namespace tutorial {


class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

protected:

private:
    // void tick();

    

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_fruit_pubs_;

    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    
    void switch_team(bool blue_team);


    // rclcpp::TimerBase::SharedPtr tick_timer_;

    // bool blue_team_ = false;
    // ::params::LocalROS2ParamProvider blue_team;
};

}  // namespace tutorial
