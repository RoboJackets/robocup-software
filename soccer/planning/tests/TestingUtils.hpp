#pragma once

#include <random>

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
template <typename T>
T random(std::mt19937& generator, T lo, T hi) {
    static std::uniform_real_distribution<> randDistribution(lo, hi);
    return randDistribution(generator);
}

/**
 * generate a random RobotInstant inside the field
 * @return random robot instant
 */
RobotInstant randomInstant(std::mt19937& generator);
}  // namespace Planning::TestingUtils