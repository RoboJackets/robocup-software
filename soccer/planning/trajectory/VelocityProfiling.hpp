#pragma once

#include <planning/RotationConstraints.hpp>
#include "PathSmoothing.hpp"
#include "Trajectory.hpp"

namespace Planning {

/**
 * Returns target angle from (position, linear velocity, previous angle)
 */
using AngleFunction = std::function<double(const RobotInstant& instant)>;

namespace AngleFns {
    inline double tangent(const RobotInstant& instant) {
        return instant.velocity.linear().angle();
    };
    inline AngleFunction facePoint(const Geometry2d::Point point) {
        return [=](const RobotInstant& instant) -> double {
            return instant.pose.position().angleTo(point);
        };
    }
    inline AngleFunction faceAngle(double angle) {
        return [=](const RobotInstant& instant) -> double {
            return angle;
        };
    }
    inline double zero(const RobotInstant& instant) {
        return 0;
    }
}

/**
 * Create a trajectory with the given path by calculating the maximum possible
 * velocity at each points given constraints on speed and acceleration.
 *
 * @param path
 * @return
 */
Trajectory ProfileVelocity(const BezierPath& path, double initial_speed, double final_speed, const MotionConstraints& constraints, RJ::Time initial_time = RJ::now());

/**
 * Create a path starting at the end of the given trajectory using the given
 * Bezier path. The resulting path will be smooth, assuming that the Bezier
 * path has matching beginning tangent and position. The resulting trajectory
 * will be appended to the given trajectory.
 *
 * @param out The trajectory to which the new path should be appended
 * @param path The (spatial) Bezier path to follow after the trajectory is finished
 * @param final_speed The final speed along the path, once the end is reached
 * @param constraints Constraints on linear acceleration.
 */
void AppendProfiledVelocity(Trajectory& out, const BezierPath& path, double final_speed, const MotionConstraints& constraints);

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
void PlanAngles(Trajectory& trajectory,
                const RobotInstant& start_instant,
                const AngleFunction& angle,
                const RotationConstraints& constraints);

} // namespace Planning
