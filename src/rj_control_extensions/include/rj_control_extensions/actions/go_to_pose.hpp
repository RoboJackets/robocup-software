/**
 * @file go_to_pose.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief Extensions on the GoToPose Action message in rj_control_msgs
 * @version 0.1
 * @date 2026-01-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <rj_geometry/pose.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_control_msgs/msg/go_to_pose.hpp>

namespace action {

/**
 * @brief The Go To Pose action is called to go to a specified pose
 * 
 */
class GoToPose {
public:
    using Msg = rj_control_msgs::msg::GoToPose;

    /**
     * @brief Construct a new Go To Pose Action
     * 
     * @param target The target pose to go to
     * @param avoid_ball Should the ball be avoided in getting to the pose
     */
    GoToPose(
        rj_geometry::Pose target,
        bool avoid_ball,
        double position_tolerance = 0.05, //NOLINT(bugprone-easily-swappable-parameters)
        double angular_tolerance = M_PI / 16
    )
        : target_(target),
          avoid_ball_(avoid_ball),
          position_tolerance_(position_tolerance),
          angular_tolerance_(angular_tolerance) {}

    /**
     * @brief Get the target pose of the action
     * 
     * @return rj_geometry::Pose 
     */
    [[nodiscard]] rj_geometry::Pose target() const {
        return target_;
    }

    /**
     * @brief Set the target pose for this action
     * 
     * @param target 
     */
    void set_target(rj_geometry::Pose target) {
        target_ = target;
    }

    /**
     * @brief Should this action avoid the ball
     * 
     * @return true 
     * @return false 
     */
    [[nodiscard]] bool avoid_ball() const {
        return avoid_ball_;
    }

    /**
     * @brief Set whether the action should avoid the ball
     * 
     * @param avoid_ball 
     */
    void set_avoid_ball(bool avoid_ball) {
        avoid_ball_ = avoid_ball;
    }

    /**
     * @brief Get the positional tolerance of the action
     * 
     * @return double 
     */
    [[nodiscard]] double position_tolerance() const {
        return position_tolerance_;
    }

    /**
     * @brief Set the position tolerance of the action
     * 
     * @param position_tolerance 
     */
    void set_position_tolerance(double position_tolerance) {
        position_tolerance_ = position_tolerance;
    }
    
    /**
     * @brief Get the angular tolerance of the action
     * 
     * @return double 
     */
    [[nodiscard]] double angular_tolerance() const {
        return angular_tolerance_;
    }

    /**
     * @brief Set the angular tolerance of the action
     * 
     * @param angular_tolerance 
     */
    void set_angular_tolerance(double angular_tolerance) {
        angular_tolerance_ = angular_tolerance;
    }

private:
    // The target pose to get to
    rj_geometry::Pose target_;
    // Should the controller treat the ball as an obstacle
    bool avoid_ball_;
    // The positional tolerance (in meters)
    double position_tolerance_ = 0.05;
    // The angular tolerance (in radians)
    double angular_tolerance_ = M_PI / 16;
};

} // namespace action

namespace rj_convert {

template <>
struct RosConverter<action::GoToPose, action::GoToPose::Msg> {
    static rj_control_msgs::msg::GoToPose to_ros(const action::GoToPose& from) {
        rj_control_msgs::msg::GoToPose msg;
        msg.target = convert_to_ros(from.target());
        msg.avoid_ball = from.avoid_ball();
        msg.position_tolerance = from.position_tolerance();
        msg.angular_tolerance = from.angular_tolerance();
        return msg;
    }

    static action::GoToPose from_ros(const rj_control_msgs::msg::GoToPose& from) {
        return {
            convert_from_ros(from.target),
            from.avoid_ball,
            from.position_tolerance,
            from.angular_tolerance
        };
    }
};

ASSOCIATE_CPP_ROS(action::GoToPose, action::GoToPose::Msg);

} // namespace rj_convert