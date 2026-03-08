#pragma once

#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/alive_robots.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_control_extensions/control_command.hpp>

namespace radio {

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
     * @param control_command The control command for the robot
     */
    virtual void send_control_message(
        uint8_t robot_id,
        const control::ControlCommand& control_command
    ) = 0;

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

    // Ros publishers to send robot statuses
    std::array<rclcpp::Publisher<rj_msgs::msg::RobotStatus>::SharedPtr, kNumShells>
        robot_status_pubs_;

    // Ros publisher to update alive robots
    rclcpp::Publisher<rj_msgs::msg::AliveRobots>::SharedPtr alive_robots_pub_;

    // Ros subscribers to receive control commands to send to the robot
    std::array<rclcpp::Subscription<control::ControlCommand::Msg>::SharedPtr, kNumShells>
        control_subs_;
    // Cached Control Commands
    std::array<control::ControlCommand, kNumShells> control_commands_;
    // Last Update Timestamps (per robot)
    std::array<RJ::Time, kNumShells> last_updates_ = {};

    // Ros subscriber for the team's color.
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    // Whether or not the current team color is blue
    bool blue_team_;
};

}  // namespace radio
