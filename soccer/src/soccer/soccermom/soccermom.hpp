#pragma once

#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/alive_robots.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/motion_setpoint.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_param_utils/param.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>

#include "robot_intent.hpp"
#include "std_msgs/msg/string.hpp"
#include "strategy/agent/position/positions.hpp"

/**
 * @brief Sends and receives information to/from our robots.
 *
 * @details This is the abstract superclass for NetworkRadio and SimRadio, which do
 * the actual work - this just declares the interface and handles sending stop commands when no new
 * commands come in for a while.
 *
 * The radio should handle:
 *  1. Sending Control Messages to the Robots
 *      * Alive Robots should be receiving real commands
 *      * Non-responsive (dead) robots should receive stop commands
 * 2. Receiving Robot Status Messages and Publish to the robot status topic
 * 3. Calculate Alive Robots and Publish to the alive robots topic
 */
class Soccermom : public rclcpp::Node {
public:
    Soccermom(bool blue_team = false);

protected:
    // Time between consecutive calls to tick().
    std::chrono::milliseconds tick_period_ = std::chrono::milliseconds(100);
    // Ros timer to trigger tick every tick_period
    rclcpp::TimerBase::SharedPtr tick_timer_;

    // The position of each robot
    std::array<strategy::Positions, kNumShells> positions_;

    // Ros publishers to send robot statuses
    std::array<rclcpp::Publisher<rj_msgs::msg::RobotStatus>::SharedPtr, kNumShells>
        robot_status_pubs_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr team_fruit_pub_;

    // Ros publisher to update alive robots
    rclcpp::Publisher<rj_msgs::msg::AliveRobots>::SharedPtr alive_robots_pub_;

    // Ros subscribers to receive velocity commands, which are sent to the robot
    std::array<rclcpp::Subscription<rj_msgs::msg::MotionSetpoint>::SharedPtr, kNumShells>
        motion_subs_;
    // Last Update Timestamps (per robot)
    std::array<RJ::Time, kNumShells> last_updates_ = {};
    // Cached last velocity command
    std::array<rj_msgs::msg::MotionSetpoint::SharedPtr, kNumShells> motions_;

    // Ros subscribers to receive auxillary control (i.e. shoot_mode, trigger_mode, kick_speed, and
    // dribbler_speed) which are stored and sent to the robot
    std::array<rclcpp::Subscription<rj_msgs::msg::ManipulatorSetpoint>::SharedPtr, kNumShells>
        manipulator_subs_;
    // Cached auxillary control information
    std::array<rj_msgs::msg::ManipulatorSetpoint, kNumShells> manipulators_cached_;

    // Ros subscriber for the team's color.
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    // Whether or not the current team color is blue
    bool blue_team_;
};