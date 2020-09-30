#pragma once

#include <rj_geometry/point.hpp>
#include <rj_geometry/pose.hpp>
#include <optional>
#include <rj_common/time.hpp>
#include <rj_msgs/msg/linear_motion_instant.hpp>
#include <rj_msgs/msg/robot_instant.hpp>
#include <rj_geometry/geometry_conversions.hpp>

#include "debug_drawer.hpp"
#include "planning/dynamic_obstacle.hpp"

namespace Planning {

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

}  // namespace Planning

namespace rj_convert {

template <>
struct RosConverter<Planning::LinearMotionInstant, rj_msgs::msg::LinearMotionInstant> {
    static rj_msgs::msg::LinearMotionInstant to_ros(const Planning::LinearMotionInstant& from) {
        return rj_msgs::build<rj_msgs::msg::LinearMotionInstant>()
            .position(convert_to_ros(from.position))
            .velocity(convert_to_ros(from.velocity));
    }

    static Planning::LinearMotionInstant from_ros(const rj_msgs::msg::LinearMotionInstant& from) {
        return Planning::LinearMotionInstant{convert_from_ros(from.position), convert_from_ros(from.velocity)};
    }
};

ASSOCIATE_CPP_ROS(Planning::LinearMotionInstant, Planning::LinearMotionInstant::Msg);

template <>
struct RosConverter<Planning::RobotInstant, rj_msgs::msg::RobotInstant> {
    static rj_msgs::msg::RobotInstant to_ros(const Planning::RobotInstant& from) {
        return rj_msgs::build<rj_msgs::msg::RobotInstant>()
            .stamp(convert_to_ros(from.stamp))
            .pose(convert_to_ros(from.pose))
            .velocity(convert_to_ros(from.velocity));
    }

    static Planning::RobotInstant from_ros(const rj_msgs::msg::RobotInstant& from) {
        return Planning::RobotInstant{convert_from_ros(from.pose), convert_from_ros(from.velocity), convert_from_ros(from.stamp)};
    }
};

ASSOCIATE_CPP_ROS(Planning::RobotInstant, Planning::RobotInstant::Msg);


} // namespace rj_convert
