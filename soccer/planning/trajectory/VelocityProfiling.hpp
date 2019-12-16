#pragma once

#include <planning/RotationConstraints.hpp>
#include "PathSmoothing.hpp"
#include "Trajectory.hpp"

namespace Planning {

/**
 * Returns target angle from (position, linear velocity, previous angle)
 */
using AngleFunction = std::function<double(Geometry2d::Point, Geometry2d::Point, double)>;

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
 * Use an angle function to modify a trajectory with valid angles.
 *
 * @param trajectory The trajectory to fix with angles. This will be modified
 *      in-place.
 * @param start_instant The initial pose and velocity of the robot.
 * @param angle An angle function describing the desired state of the robot as
 *      a function of the robot's position and linear velocity.
 * @param constraints Constraints on the robot's rotation.
 */
void PlanAngles(Trajectory& trajectory,
                const RobotInstant& start_instant,
                const AngleFunction& angle,
                const RotationConstraints& constraints);

} // namespace Planning
