/**
 * @file rf_radio_node.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief The rj radio node runs on the base station. It is responsible for publishing commands to each of
 * the robots and receiving the status of each robot.  It uses an nRF24L01+ with the following transmission
 * pattern:
 *
 *
 * Control Message -> Robot X
 * Control Message -> Robot X+1
 * ...
 * Control Message -> Robot X+n-1
 * Robot X -> Status Message
 * Robot X+1 -> Status Message
 * ...
 * Robot X+n-1 -> Status Message
 * (Total Time: 10ms)
 *  
 * @version 0.1
 * @date 2025-12-22
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once
#pragma GCC diagnostic ignored "-Wcomment"

#include <unordered_map>
#include <unordered_set>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_msgs/msg/motion_setpoint.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/robot_status.hpp>

#include <RF24.h>
#include <pigpio.h>

#include "rj_base_station/messages.hpp"

namespace base_station {

/**
 * @brief The RF Radio Node has a radio and transmits and receives to a number of robots
 * 
 */
// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class RFRadioNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new RFRadioNode
     * 
     */
    RFRadioNode();

    ~RFRadioNode() override {
        gpioSetAlertFuncEx(irq_, nullptr, nullptr);
        gpioTerminate();
    }

private:
    /**
     * @brief Send amotion commands to each of the robots
     * 
     */
    void send_motion_commands();

    /**
     * @brief Publish the robot statuses received into the ros system
     * 
     */
    void publish_robot_statuses();

    static void gpio_cb(int gpio, int level, uint32_t tick, void * user);

    /**
     * @brief GPIO ISR for when the radio receives data
     * 
     */
    void radio_gpio_callback();

    /**
     * @brief Decode the power amplifier value from a string
     * 
     * @param str The string representing the power amplifier value
     * @return rf24_pa_dbm_e The power amplifier enum value
     */
    static rf24_pa_dbm_e decode_pa(const std::string& str);

    /**
     * @brief Parse the set of updated parameters
     * 
     * @param params The updated parameters
     * @return true If the parameters were set successfully
     * @return false If the parameters were not set successfully
     */
    bool parse_parameters(const std::vector<rclcpp::Parameter>& params);

    // Handle to the callback for when parameters change
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> param_cb_handle_;
    // The robots the radio is responsible for transmitting to
    std::vector<int64_t> transmit_robots_ = {};

    // A subscription to the motion setpoints for each of the robots this node is responsible
    // for transmitting to
    std::unordered_map<int64_t, std::shared_ptr<rclcpp::Subscription<rj_msgs::msg::MotionSetpoint>>> motion_subs_;
    // The most recent motion setpoint for each robot we're responsible for transmitting to
    std::unordered_map<int64_t, std::shared_ptr<rj_msgs::msg::MotionSetpoint>> motion_setpoints_;
    // A subscription to the manipulator setpoints for each of the robots this node is responsible
    // for transmitting to
    std::unordered_map<int64_t, std::shared_ptr<rclcpp::Subscription<rj_msgs::msg::ManipulatorSetpoint>>> manipulator_subs_;
    // The most recent maniupulator setpoint for each robot we're responsible for transmitting to
    std::unordered_map<int64_t, std::shared_ptr<rj_msgs::msg::ManipulatorSetpoint>> manipulator_setpoints_;
    // The publishers for the status of each robot this radio is responsible for
    std::unordered_map<int64_t, std::shared_ptr<rclcpp::Publisher<rj_msgs::msg::RobotStatus>>> robot_status_pubs_;
    // Queue of robot status messages received (in cpp form (not ros))
    std::unordered_map<int64_t, std::optional<rtp::RobotStatusMessage>> robot_status_queue_ = {};

    // A subscription to the current team color
    std::shared_ptr<rclcpp::Subscription<rj_msgs::msg::TeamColor>> team_color_sub_;
    // Is the current team color blue?
    bool blue_team_;

    // Timer to transmit the most recent set of commands
    std::shared_ptr<rclcpp::TimerBase> transmit_timer_;

    // The nRF24l01+ radio
    RF24 radio_;
    // The interrupt pin
    uint8_t irq_;
};

} // namespace base_station