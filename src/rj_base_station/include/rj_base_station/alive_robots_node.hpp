/**
 * @file alive_robots_node.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief The alive robots node publishes what robots are considered alive (according to the base station)
 * @version 0.1
 * @date 2026-03-08
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <fmt/format.h>

#include <rj_constants/topic_names.hpp>
#include <rj_constants/constants.hpp>
#include <rj_msgs/msg/alive_robots.hpp>
#include <rj_msgs/msg/robot_status.hpp>

namespace base_station {

class AliveRobotsNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new AliveRobotsNode
     * 
     */
    AliveRobotsNode();

private:
    // The amount of time between subsequent alive robots calculations
    static constexpr std::chrono::milliseconds kTickPeriod = std::chrono::milliseconds(500);

    /**
     * @brief Timer callback to periodically publish the alive robots
     * 
     */
    void publish_alive_robots();

    // A subscription to the status of each robot
    std::array<std::shared_ptr<rclcpp::Subscription<rj_msgs::msg::RobotStatus>>, kNumShells> robot_status_subs_ = {};
    // The last update timestamp of each robot's status
    std::array<std::chrono::system_clock::time_point, kNumShells> last_updates_ = {};
    // Publisher to publish the list of alive robots
    std::shared_ptr<rclcpp::Publisher<rj_msgs::msg::AliveRobots>> alive_robots_pub_;
    // Timer to schedule the publishing of alive robots
    std::shared_ptr<rclcpp::TimerBase> tick_timer_;

    // Handle to the callback for when parameters change
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> parameter_callback_handle_;
    // The amount of time (in milliseconds) before considering a robot dead
    std::chrono::milliseconds death_timeout_ = std::chrono::milliseconds(500);
};

} // namespace base_station