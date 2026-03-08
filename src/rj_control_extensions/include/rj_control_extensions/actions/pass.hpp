/**
 * @file pass.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief Extensions on the Pass Action Message in rj_control_msgs
 * @version 0.1
 * @date 2026-01-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <rj_convert/ros_convert.hpp>
#include <rj_control_msgs/msg/pass.hpp>

namespace action {

/**
 * @brief The pass action is called to pass the ball directly to a given robot
 * 
 */
class Pass {
public:
    using Msg = rj_control_msgs::msg::Pass;

    /**
     * @brief Construct a new Pass Action
     * 
     * @param robot_id The robot to pass to
     * @param power The power to pass with
     */
    //NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    Pass(int robot_id, double power)
        : robot_id_(robot_id), power_(power) {}

    /**
     * @brief Get the id of the robot to pass to
     * 
     * @return int 
     */
    [[nodiscard]] int robot_id() const {
        return robot_id_;
    }

    /**
     * @brief Set the robot id to pass to
     * 
     * @param robot_id 
     */
    void set_robot_id(int robot_id) {
        robot_id_ = robot_id;
    }

    /**
     * @brief Get the power to pass with
     * 
     * @return double 
     */
    [[nodiscard]] double power() const {
        return power_;
    }

    /**
     * @brief Set the power to pass with
     * 
     * @param power 
     */
    void set_power(double power) {
        power_ = power;
    }

private:
    // The id of the robot to pass to
    int robot_id_;
    // The power to kick the ball with
    double power_;
};

} // namespace action

namespace rj_convert {

template <>
struct RosConverter<action::Pass, action::Pass::Msg> {
    static rj_control_msgs::msg::Pass to_ros(const action::Pass& from) {
        rj_control_msgs::msg::Pass msg;
        msg.robot_id = from.robot_id();
        msg.power = from.power();
        return msg;
    }

    static action::Pass from_ros(const rj_control_msgs::msg::Pass& from) {
        return {
            from.robot_id,
            from.power
        };
    }
};

ASSOCIATE_CPP_ROS(action::Pass, action::Pass::Msg);

} // namespace rj_convert