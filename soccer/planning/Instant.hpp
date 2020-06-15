#pragma once

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Pose.hpp>
#include <optional>
#include <time.hpp>

#include "DebugDrawer.hpp"
#include "Utils.hpp"
#include "planning/DynamicObstacle.hpp"

namespace Planning {

/**
 * @brief This class represents a robot's motion "state" at a given time,
 * including position and velocity.
 */
struct LinearMotionInstant {
    explicit LinearMotionInstant(Geometry2d::Point pos = {0, 0},
                                 Geometry2d::Point vel = {0, 0})
        : position(pos), velocity(vel) {}

    Geometry2d::Point position;
    Geometry2d::Point velocity;

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
    RobotInstant(Geometry2d::Pose pose, Geometry2d::Twist velocity,
                 RJ::Time stamp)
        : pose(pose), velocity(velocity), stamp(stamp) {}

    RobotInstant(LinearMotionInstant linear_motion, double heading,
                 double angular_velocity)
        : pose(linear_motion.position, heading),
          velocity(linear_motion.velocity, angular_velocity) {}

    static bool nearly_equals(const RobotInstant& a, const RobotInstant& b,
                              double tolerance = 1e-6) {
        return Geometry2d::Pose::nearly_equals(a.pose, b.pose, tolerance) &&
               Geometry2d::Twist::nearly_equals(a.velocity, b.velocity,
                                                tolerance) &&
               a.stamp == b.stamp;
    }

    RobotInstant() = default;

    Geometry2d::Pose pose;
    Geometry2d::Twist velocity;
    RJ::Time stamp;

    [[nodiscard]] Geometry2d::Point& position() { return pose.position(); }
    [[nodiscard]] Geometry2d::Point position() const { return pose.position(); }
    [[nodiscard]] Geometry2d::Point& linear_velocity() {
        return velocity.linear();
    }
    [[nodiscard]] Geometry2d::Point linear_velocity() const {
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
        return pose != other.pose || velocity != other.velocity ||
               stamp != other.stamp;
    }
};

}  // namespace Planning
