/**
 * @file rotate_to_heading.hpp
 * @author Nathaniel Wert (Extensions on the Rotate to Heading message in rj_control_msgs)
 * @brief 
 * @version 0.1
 * @date 2026-01-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cmath>
#include <rj_control_msgs/msg/rotate_to_heading.hpp>
#include <rj_convert/ros_convert.hpp>

namespace action {

/**
 * @brief The Rotate to Heading action makes the robot rotate to a given heading
 * 
 */
class RotateToHeading {
public:
    using Msg = rj_control_msgs::msg::RotateToHeading;

    /**
     * @brief Construct a new Rotate To Heading action
     * 
     * @param heading The target heading
     * @param tolerance The tolerance for the rotation
     */
    //NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    RotateToHeading(double heading, double tolerance = M_PI / 16)
        : heading_(heading), tolerance_(tolerance) {}
    
    /**
     * @brief Get the heading to rotate to
     * 
     * @return double 
     */
    [[nodiscard]] double heading() const {
        return heading_;
    }

    /**
     * @brief Set the heading to rotate to
     * 
     * @param heading 
     */
    void set_heading(double heading) {
        heading_ = heading;
    }

    /**
     * @brief Get the tolerance of the rotation
     * 
     * @return double 
     */
    [[nodiscard]] double tolerance() const {
        return tolerance_;
    }

    /**
     * @brief Set the tolerance of the rotation
     * 
     * @param tolerance 
     */
    void set_tolerance(double tolerance) {
        tolerance_ = tolerance;
    }

private:
    // The heading to rotate to
    double heading_;
    // The tolerance (in radians) for the rotation
    double tolerance_ = M_PI / 16;
};

} // namespace action

namespace rj_convert {

template <>
struct RosConverter<action::RotateToHeading, action::RotateToHeading::Msg> {
    static action::RotateToHeading::Msg to_ros(const action::RotateToHeading& from) {
        action::RotateToHeading::Msg msg;
        msg.heading = from.heading();
        msg.tolerance = from.tolerance();
        return msg;
    }

    static action::RotateToHeading from_ros(const action::RotateToHeading::Msg& from) {
        return {
            from.heading,
            from.tolerance
        };
    }
};

ASSOCIATE_CPP_ROS(action::RotateToHeading, action::RotateToHeading::Msg);

} // namespace rj_convert