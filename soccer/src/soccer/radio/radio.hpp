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
 *
 * The radio should handle:
 *  1. Sending Control Messages to the Robots
 *      * Alive Robots should be receiving real commands
 *      * Non-responsive (dead) robots should receive stop commands
 * 2. Receiving Robot Status Messages and Publish to the robot status topic
 * 3. Calculate Alive Robots and Publish to the alive robots topic
 */
class Radio : public rclcpp::Node {
public:
    Radio();

protected:
    /**
     * @brief Send a control message to the corresponding robot.
     *
     * @param robot_id The robot to send to
     * @param motion The (x (m/s), y (m/s), z (rad/s)) velocities for the robot to move at
     * @param manipulator The Shoot Mode, Trigger Mode, Kick Speed, and Dribbler Speed for the Robot
     * @param role The position for the robot
     */
    virtual void send_control_message(uint8_t robot_id, const rj_msgs::msg::MotionSetpoint& motion,
                                      const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                                      strategy::Positions role) = 0;

    /**
     * @brief Poll the receiver service for Messages.
     *
     */
    virtual void poll_receive() = 0;

    /**
     * @brief Switch the team this radio is sending data to.
     *
     * @param blue_team
     */
    virtual void switch_team(bool blue_team) = 0;

    /**
     * @brief Wrapper over the local publisher to publish a robot status for a given robot
     *
     * @param robot_id The robot id of the robot, whose status to publish
     * @param robot_status The status of the robot
     */
    void publish_robot_status(int robot_id, const rj_msgs::msg::RobotStatus& robot_status);

    /**
     * @brief Wrapper over the private publisher to publish a message containing alive robots
     *
     * @param alive_robots A message containing the alive robots
     */
    void publish_alive_robots(const rj_msgs::msg::AliveRobots& alive_robots);

    bool blue_team() const;

private:
    /**
     * @brief Poll the receiver and send empty motion commands to robots that software has
     * not updated for a long time.
     *
     */
    void tick();
    // Time between consecutive calls to tick().
    std::chrono::milliseconds tick_period_ = std::chrono::milliseconds(100);
    // Ros timer to trigger tick every tick_period
    rclcpp::TimerBase::SharedPtr tick_timer_;

    // The position of each robot
    std::array<strategy::Positions, kNumShells> positions_;

    // Ros publishers to send robot statuses
    std::array<rclcpp::Publisher<rj_msgs::msg::RobotStatus>::SharedPtr, kNumShells>
        robot_status_pubs_;

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

    // Ros param provider for initializing the radio node
    ::params::LocalROS2ParamProvider param_provider_;
};

}  // namespace radio
