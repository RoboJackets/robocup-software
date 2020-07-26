#pragma once

#include "planning/Instant.hpp"
#include "planning/RobotConstraints.hpp"
#include "planning/Trajectory.hpp"

namespace Planning {

/**
 * @brief Represents a functor that calculates the next angular target given the
 * linear position and velocity as well as the (rough) previous angle. The
 * previous angle only exists to calculate a good local minimum.
 *
 * @param instant The current linear motion of the robot
 * @param previous_angle Represents a general area of the previous angle. Used
 * for selecting the correct local minimum.
 * @param jacobian Output parameter for the Jacobian of the angle with respect
 * to position. Used to calculate angular velocity.
 * @return A target angle to attempt to follow.
 */
using AngleFunction =
    std::function<double(const LinearMotionInstant& instant,
                         double previous_angle, Eigen::Vector2d* jacobian)>;

namespace AngleFns {

/**
 * @brief An angle function to move tangent to the path. `previous_angle` is
 * used to decide whether to target forwards or backwards motion.
 */
inline double tangent(const LinearMotionInstant& instant, double previous_angle,
                      Eigen::Vector2d* jacobian) {
    Geometry2d::Point vel = instant.velocity;

    double delta_forward = fixAngleRadians(vel.angle() - previous_angle);
    double delta_reverse = fixAngleRadians(M_PI + vel.angle() - previous_angle);

    if (jacobian != nullptr) {
        *jacobian = Eigen::Vector2d::Zero();
    }

    double result = 0.0;

    if (std::abs(delta_forward) < std::abs(delta_reverse)) {
        result = delta_forward + previous_angle;
    } else {
        result = delta_reverse + previous_angle;
    }

    return result;
}

/**
 * @brief Create an @ref AngleFunction for facing a particular point on the
 * field.
 */
inline AngleFunction facePoint(const Geometry2d::Point point) {
    return [=](const LinearMotionInstant& instant, double /*previous_angle*/,
               Eigen::Vector2d* jacobian) -> double {
        if (jacobian != nullptr) {
            Geometry2d::Point displacement = point - instant.position;
            double distance_sq = displacement.magsq();
            *jacobian =
                Eigen::Vector2d(displacement.rotate(-M_PI / 2) / distance_sq);
        }

        return instant.position.angleTo(point);
    };
}

/**
 * @brief Create an @ref AngleFunction for facing a particular constant angle.
 */
inline AngleFunction faceAngle(double angle) {
    return [=](const LinearMotionInstant& /*instant*/,
               double /*previous_angle*/, Eigen::Vector2d* jacobian) -> double {
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
void PlanAngles(Trajectory* trajectory, const RobotInstant& start_instant,
                const AngleFunction& angle,
                const RotationConstraints& constraints);

}  // namespace Planning