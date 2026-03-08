/**
 * @brief The Alive Robots Node runs on the base station and is responsible for publishing what robots
 * are considered alive.
 * 
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_common/time.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_constants/constants.hpp>
#include <rj_msgs/msg/alive_robots.hpp>
#include <rj_msgs/msg/robot_status.hpp>

namespace base_station {

class AliveRobotsNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Alive Robots Node
     * 
     */
    AliveRobotsNode();

private:
    // The amount of time between subsequent alive robots calculations  
    static constexpr std::chrono::milliseconds kTickPeriod = std::chrono::milliseconds(500);
    // The amount of time before we consider a robot dead
    static constexpr std::chrono::milliseconds kDeathTimeout = std::chrono::milliseconds(500);

    /**
     * @brief Timer callback to periodically publish the alive robots
     * 
     */
    void publish_alive_robots();

    // A subscription to the status of each robot
    std::array<std::shared_ptr<rclcpp::Subscription<rj_msgs::msg::RobotStatus>>, kNumShells> robot_status_subs_ = {};
    // The last update timestamp of each robot's status
    std::array<RJ::Time, kNumShells> last_updates_ = {};
    // Publisher to publish the alive robots
    std::shared_ptr<rclcpp::Publisher<rj_msgs::msg::AliveRobots>> alive_robots_pub_;
    // Timer to schedule the publishing of alive robots
    std::shared_ptr<rclcpp::TimerBase> tick_timer_;
};
    
} // namespace base_station