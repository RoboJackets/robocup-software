#pragma once

#include <optional>

#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_msgs/msg/linear_motion_instant.hpp>
#include <rj_msgs/msg/robot_instant.hpp>

#include "debug_drawer.hpp"
#include "planning/dynamic_obstacle.hpp"

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

namespace rj_convert {

template <>
struct RosConverter<planning::LinearMotionInstant, rj_msgs::msg::LinearMotionInstant> {
    static rj_msgs::msg::LinearMotionInstant to_ros(const planning::LinearMotionInstant& from) {
        return rj_msgs::build<rj_msgs::msg::LinearMotionInstant>()
            .position(convert_to_ros(from.position))
            .velocity(convert_to_ros(from.velocity));
    }

    static planning::LinearMotionInstant from_ros(const rj_msgs::msg::LinearMotionInstant& from) {
        return planning::LinearMotionInstant{convert_from_ros(from.position),
                                             convert_from_ros(from.velocity)};
    }
};

ASSOCIATE_CPP_ROS(planning::LinearMotionInstant, planning::LinearMotionInstant::Msg);

template <>
struct RosConverter<planning::RobotInstant, rj_msgs::msg::RobotInstant> {
    static rj_msgs::msg::RobotInstant to_ros(const planning::RobotInstant& from) {
        return rj_msgs::build<rj_msgs::msg::RobotInstant>()
            .stamp(convert_to_ros(from.stamp))
            .pose(convert_to_ros(from.pose))
            .velocity(convert_to_ros(from.velocity));
    }

    static planning::RobotInstant from_ros(const rj_msgs::msg::RobotInstant& from) {
        return planning::RobotInstant{convert_from_ros(from.pose), convert_from_ros(from.velocity),
                                      convert_from_ros(from.stamp)};
    }
};

ASSOCIATE_CPP_ROS(planning::RobotInstant, planning::RobotInstant::Msg);

}  // namespace rj_convert
