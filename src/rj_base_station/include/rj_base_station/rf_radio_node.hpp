/**
 * @file rf_radio_node.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief The rf radio node is responsible for publishing commands to each of the robots and
 * receiving status updates from the robots. It uses the RF24 library to communicate with the
 * robots over the 2.4GHz radio frequency.
 * @version 0.1
 * @date 2026-03-08
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once
#pragma GCC diagnostic ignored "-Wcomment"

#include <atomic>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>

#include <RF24.h>
#include <gpiod.h>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/motion_setpoint.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_msgs/msg/team_color.hpp>

#include "rj_base_station/messages.hpp"

namespace base_station {

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class RFRadioNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new RFRadioNode
     *
     */
    RFRadioNode();

    ~RFRadioNode() override;

private:
    /**
     * @brief Send motion commands to each of the robots
     *
     */
    void send_motion_commands();

    /**
     * @brief Publish the robot statuses received from the robots as ROS messages
     *
     */
    void publish_robot_statuses();

    /**
     * @brief Interrupt loop that runs waiting for a gpio interrupt from the radio
     *
     */
    void irq_loop();

    /**
     * @brief GPIO ISR for when the radio receives data
     *
     */
    void radio_gpio_callback();

    /**
     * @brief Parse the radio id from the node's ros2 namespace of form "radio_N"
     *
     * @param ns This node's namespace
     * @return uint8_t The id of the robot
     */
    static uint8_t parse_radio_id(const std::string& ns);  // NOLINT(readability-identifier-length)

    /**
     * @brief Decode the power amplifier value from a string
     *
     * @param str The string representing the power amplifier value
     * @return rf24_pa_dbm_e The power amplifier enum value
     */
    static rf24_pa_dbm_e decode_pa(const std::string& str);

    /**
     * @brief Parse the set of updated parameters from ROS
     *
     * @param params The ROS parameters that were updated
     * @return true True if the parameters were set successfully
     * @return false False if the parameters were not set successfully
     */
    bool parse_parameters(const std::vector<rclcpp::Parameter>& params);

    // Handle to the callback for when parameters change
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle>
        parameter_callback_handle_;
    // The robots the radio is responsible for transmitting to
    std::vector<int64_t> transmit_robots_ = {};

    // A subscription to the motion setpoints for each robot
    std::unordered_map<int64_t, std::shared_ptr<rclcpp::Subscription<rj_msgs::msg::MotionSetpoint>>>
        motion_subs_;
    // The most recent motion setpoint for each robot
    std::unordered_map<int64_t, std::shared_ptr<rj_msgs::msg::MotionSetpoint>> motion_setpoints_;
    // A subscription to the manipulator setpoints for each robot
    std::unordered_map<int64_t,
                       std::shared_ptr<rclcpp::Subscription<rj_msgs::msg::ManipulatorSetpoint>>>
        manipulator_subs_;
    // The most recent manipulator setpoint for each robot
    std::unordered_map<int64_t, std::shared_ptr<rj_msgs::msg::ManipulatorSetpoint>>
        manipulator_setpoints_;

    // A publisher for the robot statuses of each robot
    std::unordered_map<int64_t, std::shared_ptr<rclcpp::Publisher<rj_msgs::msg::RobotStatus>>>
        status_pubs_;
    // Queue of rtp robot status messages received (in cpp form (not ros))
    std::unordered_map<int64_t, std::optional<rtp::RobotStatusMessage>> robot_statuses_ = {};

    // A subscription to the current team color
    std::shared_ptr<rclcpp::Subscription<rj_msgs::msg::TeamColor>> team_color_sub_;
    // Is the current team color blue?
    bool blue_team_ = false;

    // Timer to transmit the most recent set of commands
    std::shared_ptr<rclcpp::TimerBase> transmit_timer_;

    // The nRF24l01+ radio
    RF24 radio_;
    // The nrf24l01+ mutex
    std::mutex rf24_mutex_;
    // The id of the radio
    uint8_t radio_id_;
    // The interrupt pin
    uint8_t irq_;

    // The gpiod chip
    gpiod_chip* chip_ = nullptr;
    // The irq line
    gpiod_line* irq_line_ = nullptr;
    // The interrupt thread
    std::thread irq_thread_;
    // Is the interrupt running
    std::atomic<bool> running_;
};

}  // namespace base_station
