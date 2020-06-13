#pragma once

#include "planning/Instant.hpp"
#include "planning/RobotConstraints.hpp"
#include "planning/Trajectory.hpp"

namespace Planning {

/**
 * Returns target angle from (position, linear velocity, previous angle)
 */
using AngleFunction = std::function<double(const LinearMotionInstant& instant, double previous_angle, Eigen::Vector2d* jacobian)>;

namespace AngleFns {

inline double tangent(const LinearMotionInstant& instant,
                      double previous_angle,
                      Eigen::Vector2d* jacobian) {
    Geometry2d::Point vel = instant.velocity;

    double delta_forward = fixAngleRadians(vel.angle() - previous_angle);
    double delta_reverse = fixAngleRadians(M_PI + vel.angle() - previous_angle);

    if (std::abs(delta_forward) < std::abs(delta_reverse)) {
        return delta_forward + previous_angle;
    } else {
        return delta_reverse + previous_angle;
    }
}

inline AngleFunction facePoint(const Geometry2d::Point point) {
    return [=](const LinearMotionInstant& instant,
               double /*previous_angle*/,
               Eigen::Vector2d* jacobian) -> double {
        if (jacobian != nullptr) {
            Geometry2d::Point displacement = point - instant.position;
            double distance_sq = displacement.magsq();
            *jacobian = Eigen::Vector2d(
                displacement.rotate(-M_PI / 2) / distance_sq);
        }

        return instant.position.angleTo(point);
    };
}

inline AngleFunction faceAngle(double angle) {
    return [=](const LinearMotionInstant&  /*instant*/,
               double /*previous_angle*/,
               Eigen::Vector2d* jacobian) -> double {
        if (jacobian != nullptr) {
            *jacobian = Eigen::Vector2d::Zero();
        }

        return angle;
    };
}

}  // namespace AngleFns

/**
 * Applies an angle function to a given trajectory and enforces the angle
 * constraints. This function may add instants to the end of the trajectory to
 * allow the angle to match the output from angle_function
 * (assumes the target angular velocity at the end of the path is 0)
 *
 * @param trajectory The trajectory to plan angles on
 * @param start_instant The initial robot instant
 * @param angle An angle function describing the desired angle of the robot as
 *      a function of the robot's current state.
 * @param constraints Constraints on the robot's rotation.
 */
void PlanAngles(Trajectory* trajectory,
                const RobotInstant& start_instant,
                const AngleFunction& angle,
                const RotationConstraints& constraints);

} // namespace Planning