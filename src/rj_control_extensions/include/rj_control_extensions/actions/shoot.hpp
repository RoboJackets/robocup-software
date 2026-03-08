/**
 * @file shoot.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief Extensions on the Shoot Action message in rj_control_msgs
 * @version 0.1
 * @date 2026-01-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <rj_convert/ros_convert.hpp>
#include <rj_control_msgs/msg/shoot.hpp>

namespace action {

/**
 * @brief The Shoot action is used to line kick the ball at the goal
 * 
 */
class Shoot {
public:
    using Msg = rj_control_msgs::msg::Shoot;

    /**
     * @brief Construct a new Shoot Action
     * 
     * @param goal_location The location within the goal to shoot at offset from the center in meters
     * @param power The power to kick with
     */
    //NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    Shoot(double goal_location, double power)
        : goal_location_(goal_location), power_(power) {}

    /**
     * @brief Get the location (offset from the center) to shoot at
     * 
     * @return double 
     */
    [[nodiscard]] double goal_location() const {
        return goal_location_;
    }

    /**
     * @brief Set the location (offset from the center) to shoot at
     * 
     * @param goal_location 
     */
    void set_goal_location(double goal_location) {
        goal_location_ = goal_location;
    }

    /**
     * @brief Get the power to shoot the ball with
     * 
     * @return double 
     */
    [[nodiscard]] double power() const {
        return power_;
    }

    /**
     * @brief Set the power to shoot the ball with
     * 
     * @param power 
     */
    void set_power(double power) {
        power_ = power;
    }

private:
    // The position in the goal (from -1 to 1) to shoot the ball at
    double goal_location_;
    // The power to kick the ball with
    double power_;
};

} // namespace action

namespace rj_convert {

template <>
struct RosConverter<action::Shoot, action::Shoot::Msg> {
    static rj_control_msgs::msg::Shoot to_ros(const action::Shoot& from) {
        rj_control_msgs::msg::Shoot msg;
        msg.goal_location = from.goal_location();
        msg.power = from.power();
        return msg;
    }

    static action::Shoot from_ros(const rj_control_msgs::msg::Shoot& from) {
        return {
            from.goal_location,
            from.power
        };
    }
};

ASSOCIATE_CPP_ROS(action::Shoot, action::Shoot::Msg);

} // namespace rj_convert