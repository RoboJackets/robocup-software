/**
 * @file clear.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief Extensions on the Clear Action message in rj_control_msgs
 * @version 0.1
 * @date 2026-01-03
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <rj_geometry/point.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_control_msgs/msg/clear.hpp>

namespace action {

/**
 * @brief The clear action is called to chip to ball to a specified location
 * 
 */
class Clear {
public:
    using Msg = rj_control_msgs::msg::Clear;
    
    /**
     * @brief Construct a new Clear Action
     * 
     * @param clear_target The target point to clear to
     * @param power The power to use to clear
     */
    Clear(rj_geometry::Point clear_target, double power)
        : clear_target_(clear_target), power_(power) {}

    /**
     * @brief Get the target clear point
     * 
     * @return rj_geometry::Point 
     */
    [[nodiscard]] rj_geometry::Point clear_target() const {
        return clear_target_;
    }

    /**
     * @brief Set the clear target
     * 
     * @param clear_target 
     */
    void set_clear_target(rj_geometry::Point clear_target) {
        clear_target_ = clear_target;
    }

    /**
     * @brief Get the power to be used to clear
     * 
     * @return double 
     */
    [[nodiscard]] double power() const {
        return power_;
    }

    /**
     * @brief Set the power to be used to clear
     * 
     * @param power 
     */
    void set_power(double power) {
        power_ = power;
    }

private:
    
    // The target point to clear to
    rj_geometry::Point clear_target_;
    // The power to use to clear the ball
    double power_;
};

} // namespace action

namespace rj_convert {

template <>
struct RosConverter<action::Clear, rj_control_msgs::msg::Clear> {
    static rj_control_msgs::msg::Clear to_ros(const action::Clear& from) {
        rj_control_msgs::msg::Clear clear;
        clear.clear_target = convert_to_ros(from.clear_target());
        clear.power = from.power();
        return clear;
    }

    static action::Clear from_ros(const rj_control_msgs::msg::Clear& from) {
        return {
            convert_from_ros(from.clear_target),
            from.power
        };
    }
};

ASSOCIATE_CPP_ROS(action::Clear, action::Clear::Msg);

} // namespace rj_convert
