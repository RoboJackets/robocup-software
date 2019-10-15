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
Trajectory ProfileVelocity(const BezierPath& path, double initial_speed, double final_speed, const MotionConstraints& constraints);

/**
 * Use an angle function to modify a trajectory with valid angles.
 *
 * @param trajectory The trajectory to fix with angles. This will be modified
 *      in-place.
 * @param initial_state The initial pose and velocity of the robot.
 * @param angle An angle function describing the desired state of the robot as
 *      a function of the robot's position and linear velocity.
 * @param constraints Constraints on the robot's rotation.
 */
void PlanAngles(Trajectory& trajectory,
                RobotState initial_state,
                const AngleFunction& angle,
                const RotationConstraints& constraints);

} // namespace Planning
