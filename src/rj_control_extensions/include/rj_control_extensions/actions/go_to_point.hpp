/**
 * @file go_to_point.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief Extensions on the GoToPoint Action message in rj_control_msgs
 * @version 0.1
 * @date 2026-01-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <rj_geometry/point.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_control_msgs/msg/go_to_point.hpp>

namespace action {

/**
 * @brief The go to point action is used to move the robot to a given point
 * 
 */
class GoToPoint {
public:
    using Msg = rj_control_msgs::msg::GoToPoint;

    /**
     * @brief Construct a new Go To Point Action
     * 
     * @param target The target point to go to
     * @param avoid_ball Should the ball be avoided
     */
    GoToPoint(rj_geometry::Point target, bool avoid_ball, double tolerance = 0.05)
        : target_(target), avoid_ball_(avoid_ball), tolerance_(tolerance) {}

    /**
     * @brief Get the target point
     * 
     * @return rj_geometry::Point 
     */
    [[nodiscard]] rj_geometry::Point target() const {
        return target_;
    }

    /**
     * @brief Set the target point of the action
     * 
     * @param target 
     */
    void set_target(rj_geometry::Point target) {
        target_ = target;
    }

    /**
     * @brief Get whether the movement will avoid the ball
     * 
     * @return true 
     * @return false 
     */
    [[nodiscard]] bool avoid_ball() const {
        return avoid_ball_;
    }

    /**
     * @brief Set whether the movement will avoid the ball
     * 
     * @param avoid_ball 
     */
    void set_avoid_ball(bool avoid_ball) {
        avoid_ball_ = avoid_ball;
    }

    /**
     * @brief Get the tolerance for the movemnet in meters
     * 
     * @return double 
     */
    [[nodiscard]] double tolerance() const {
        return tolerance_;
    }

    /**
     * @brief Set the tolerance of completion for the motion (in meters)
     * 
     * @param tolerance 
     */
    void set_tolerance(double tolerance) {
        tolerance_ = tolerance;
    }

private:
    // The point to move to
    rj_geometry::Point target_;
    // Should the ball be treated as an obstacle
    bool avoid_ball_;
    // The tolerance (in m from the target location)
    double tolerance_ = 0.05;
};

} // namespace action

namespace rj_convert {

template <>
struct RosConverter<action::GoToPoint, rj_control_msgs::msg::GoToPoint> {
    static rj_control_msgs::msg::GoToPoint to_ros(const action::GoToPoint& from) {
        rj_control_msgs::msg::GoToPoint msg;
        msg.target = convert_to_ros(from.target());
        msg.avoid_ball = from.avoid_ball();
        msg.tolerance = from.tolerance();
        return msg;
    }

    static action::GoToPoint from_ros(const rj_control_msgs::msg::GoToPoint& from) {
        return {
            convert_from_ros(from.target),
            from.avoid_ball,
            from.tolerance
        };
    }
};

ASSOCIATE_CPP_ROS(action::GoToPoint, action::GoToPoint::Msg);

} // namespace rj_convert