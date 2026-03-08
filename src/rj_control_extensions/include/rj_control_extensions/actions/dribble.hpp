/**
 * @file dribble.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief Extensions on the Dribble Action msessage in rj_control_msgs
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
#include <rj_control_msgs/msg/dribble.hpp>

namespace action {

/**
 * @brief The dribble action is used to dribble the ball to a specified pose
 * 
 */
class Dribble {
public:
    using Msg = rj_control_msgs::msg::Dribble;

    /**
     * @brief Construct a new Dribble Action
     * 
     * @param target The target to dribble to
     */
    Dribble(rj_geometry::Pose target)
        : target_(target) {}

    /**
     * @brief Get the target to dribble to
     * 
     * @return rj_geometry::Pose 
     */
    [[nodiscard]] rj_geometry::Pose target() const {
        return target_;
    }

    /**
     * @brief Set the target to dribble to
     * 
     * @param target 
     */
    void set_target(rj_geometry::Pose target) {
        target_ = target;
    }

private:
    // The target pose to reach
    rj_geometry::Pose target_;
};

} // namespace action

namespace rj_convert {

template <>
struct RosConverter<action::Dribble, rj_control_msgs::msg::Dribble> {
    static rj_control_msgs::msg::Dribble to_ros(const action::Dribble& from) {
        rj_control_msgs::msg::Dribble msg;
        msg.target = convert_to_ros(from.target());
        return msg;
    }

    static action::Dribble from_ros(const rj_control_msgs::msg::Dribble& from) {
        return {
            convert_from_ros(from.target)
        };
    }
};

ASSOCIATE_CPP_ROS(action::Dribble, action::Dribble::Msg);

} // namespace rj_convert