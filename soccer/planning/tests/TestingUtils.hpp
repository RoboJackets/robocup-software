#pragma once

#include "planning/Instant.hpp"
#include "planning/RobotConstraints.hpp"
#include "planning/Trajectory.hpp"

namespace Planning::TestingUtils {
/**
 * test if a path is continuous and well behaved
 * @param path
 * @param constraints
 */
bool checkTrajectoryContinuous(const Trajectory& trajectory,
                               const RobotConstraints& constraints);

/**
 * generate a random number between lo and hi
 * @param lo
 * @param hi
 * @return random number
 */
double random(double lo, double hi);

/**
 * generate a random RobotInstant inside the field
 * @return random robot instant
 */
RobotInstant randomInstant();
}  // namespace Planning::TestingUtils