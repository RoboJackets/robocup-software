#pragma once

#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/motion_setpoint.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_param_utils/param.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>

#include "robot_intent.hpp"
#include "robot_status.hpp"
#include "strategy/agent/position/positions.hpp"

namespace radio {

constexpr auto kRadioParamModule = "radio";
DECLARE_FLOAT64(kRadioParamModule, timeout);

/**
 * @brief Sends and receives information to/from our robots.
 *
 * @details This is the abstract superclass for NetworkRadio and SimRadio, which do
 * the actual work - this just declares the interface and handles sending stop commands when no new
 * commands come in for a while.
 */
class Radio : public rclcpp::Node {
public:
    Radio();

protected:
    void publish(int robot_id, const rj_msgs::msg::RobotStatus& robot_status);

    virtual void send(int robot_id, const rj_msgs::msg::MotionSetpoint& motion,
                      const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                      strategy::Positions role) = 0;
    virtual void receive() = 0;
    virtual void switch_team(bool blue) = 0;

private:
    void tick();

    std::array<strategy::Positions, kNumShells> positions_;

    std::array<rclcpp::Publisher<rj_msgs::msg::RobotStatus>::SharedPtr, kNumShells>
        robot_status_pubs_;
    std::array<rclcpp::Subscription<rj_msgs::msg::MotionSetpoint>::SharedPtr, kNumShells>
        motion_subs_;
    std::array<rclcpp::Subscription<rj_msgs::msg::ManipulatorSetpoint>::SharedPtr, kNumShells>
        manipulator_subs_;
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    rclcpp::TimerBase::SharedPtr tick_timer_;
    std::array<rj_msgs::msg::ManipulatorSetpoint, kNumShells> manipulators_cached_;
    std::array<RJ::Time, kNumShells> last_updates_ = {};

    ::params::LocalROS2ParamProvider param_provider_;
};

}  // namespace radio
