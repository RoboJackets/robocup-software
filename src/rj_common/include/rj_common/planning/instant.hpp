#pragma once

#include <optional>

#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_msgs/msg/linear_motion_instant.hpp>
#include <rj_msgs/msg/robot_instant.hpp>

#include "rj_common/debug_drawer.hpp"
#include "rj_common/planning/dynamic_obstacle.hpp"

namespace planning {

/**
 * @brief This class represents a robot's motion "state" at a given time,
 * including position and velocity.
 */
struct LinearMotionInstant {
    using Msg = rj_msgs::msg::LinearMotionInstant;

    explicit LinearMotionInstant(rj_geometry::Point pos = {0, 0},
                                 rj_geometry::Point vel = {0, 0})
        : position(pos), velocity(vel) {}

    rj_geometry::Point position;
    rj_geometry::Point velocity;

    /**
     * @brief Return true if LinearMotionInstant a and b are nearly equal.
     *
     * @param a LinearMotionInstant a
     * @param b LinearMotionInstant b
     * @param position_tolerance tolerance on position component of LinearMotionInstant
     * @param velocity_tolerance tolerance on velocity component of LinearMotionInstant
     * @return True if a and b are within position_tolerance of each other AND velocity_tolerance of
     * each other
     */
    static bool nearly_equals(const LinearMotionInstant& a, const LinearMotionInstant& b,
                              double position_tolerance = 1e-4, double velocity_tolerance = 1e-4) {
        return rj_geometry::Point::nearly_equals(a.position, b.position, position_tolerance) &&
               rj_geometry::Point::nearly_equals(a.velocity, b.velocity, velocity_tolerance);
    }

    friend std::ostream& operator<<(std::ostream& stream,
                                    const LinearMotionInstant& instant) {
        return stream << "LinearMotionInstant(position=" << instant.position
                      << ", velocity=" << instant.velocity << ")";
    }
};

/**
 * @brief Represents the current state of a robot in a planned trajectory.
 */
struct RobotInstant {
    using Msg = rj_msgs::msg::RobotInstant;

    RobotInstant(rj_geometry::Pose pose, rj_geometry::Twist velocity,
                 RJ::Time stamp)
        : pose(pose), velocity(velocity), stamp(stamp) {}

    RobotInstant(LinearMotionInstant linear_motion, double heading,
                 double angular_velocity)
        : pose(linear_motion.position, heading),
          velocity(linear_motion.velocity, angular_velocity) {}

    static bool nearly_equals(const RobotInstant& a, const RobotInstant& b,
                              double tolerance = 1e-6) {
        return rj_geometry::Pose::nearly_equals(a.pose, b.pose, tolerance) &&
               rj_geometry::Twist::nearly_equals(a.velocity, b.velocity,
                                                tolerance) &&
               a.stamp == b.stamp;
    }

    RobotInstant() = default;

    rj_geometry::Pose pose;
    rj_geometry::Twist velocity;
    RJ::Time stamp;

    [[nodiscard]] rj_geometry::Point& position() { return pose.position(); }
    [[nodiscard]] rj_geometry::Point position() const { return pose.position(); }
    [[nodiscard]] rj_geometry::Point& linear_velocity() {
        return velocity.linear();
    }
    [[nodiscard]] rj_geometry::Point linear_velocity() const {
        return velocity.linear();
    }
    [[nodiscard]] double& heading() { return pose.heading(); }
    [[nodiscard]] double heading() const { return pose.heading(); }
    [[nodiscard]] double& angular_velocity() { return velocity.angular(); }
    [[nodiscard]] double angular_velocity() const { return velocity.angular(); }

    [[nodiscard]] LinearMotionInstant linear_motion() const {
        return LinearMotionInstant{pose.position(), velocity.linear()};
    }

    /**
     * Equality comparison operator.
     */
    bool operator==(const RobotInstant& other) const {
        return pose == other.pose && velocity == other.velocity &&
               stamp == other.stamp;
    }

    /**
     * Inequality comparison operator.
     */
    bool operator!=(const RobotInstant& other) const {
        return !(*this == other);
    }
};

}  // namespace planning

namespace rclcpp {

template <>
struct TypeAdapter<planning::LinearMotionInstant, rj_msgs::msg::LinearMotionInstant> {
    using is_specialized = std::true_type;
    using custom_type = planning::LinearMotionInstant;
    using ros_message_type = rj_msgs::msg::LinearMotionInstant;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.position =
            rj_convert::convert_to_ros<rj_geometry::Point, rj_geometry_msgs::msg::Point>(
                source.position);
        destination.velocity =
            rj_convert::convert_to_ros<rj_geometry::Point, rj_geometry_msgs::msg::Point>(
                source.velocity);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination.position =
            rj_convert::convert_from_ros<rj_geometry_msgs::msg::Point, rj_geometry::Point>(
                source.position);
        destination.velocity =
            rj_convert::convert_from_ros<rj_geometry_msgs::msg::Point, rj_geometry::Point>(
                source.velocity);
    }
};


template <>
struct TypeAdapter<planning::RobotInstant, rj_msgs::msg::RobotInstant> {
    using is_specialized = std::true_type;
    using custom_type = planning::RobotInstant;
    using ros_message_type = rj_msgs::msg::RobotInstant;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.stamp =
            rj_convert::convert_to_ros<RJ::Time, builtin_interfaces::msg::Time>(source.stamp);
        destination.pose =
            rj_convert::convert_to_ros<rj_geometry::Pose, rj_geometry_msgs::msg::Pose>(
                source.pose);
        destination.velocity =
            rj_convert::convert_to_ros<rj_geometry::Twist, rj_geometry_msgs::msg::Twist>(
                source.velocity);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination.pose =
            rj_convert::convert_from_ros<rj_geometry_msgs::msg::Pose, rj_geometry::Pose>(
                source.pose);
        destination.velocity =
            rj_convert::convert_from_ros<rj_geometry_msgs::msg::Twist, rj_geometry::Twist>(
                source.velocity);
        destination.stamp =
            rj_convert::convert_from_ros<builtin_interfaces::msg::Time, RJ::Time>(source.stamp);
    }
};


}  // namespace rclcpp
