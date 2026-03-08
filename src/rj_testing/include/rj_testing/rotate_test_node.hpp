/**
 * @file rotate_test_node.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief The Rotate Test Node makes the robot rotate in place so we can tune rotational pid
 * @version 0.1
 * @date 2026-01-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <rj_utils/logging.hpp>
#include <rj_utils/parsing.hpp>
#include <rj_control_extensions/action.hpp>

namespace rj_testing {

class RotateTestNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Rotate Test Node
     * 
     */
    RotateTestNode();

private:
    // The robot id of the robot being controlled
    int robot_id_;
    // The current target heading
    double heading_ = M_PI / 2;
    
    // Publisher to publish the rotate action
    std::shared_ptr<rclcpp::Publisher<action::Action::Msg>> action_pub_;
    // Subscription to the action complete message
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> action_complete_sub_;
};

} // namespace rj_testing