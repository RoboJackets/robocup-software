#pragma once

#include <random>

#include "planning/instant.hpp"
#include "planning/robot_constraints.hpp"
#include "planning/trajectory.hpp"

namespace planning::TestingUtils {
/**
 * test if a path is continuous and well behaved
 * @param path
 * @param constraints
 */
bool check_trajectory_continuous(const Trajectory& trajectory,
                               const RobotConstraints& constraints);

/**
 * generate a random number between lo and hi
 * @param lo
 * @param hi
 * @return random number
 */
template <typename T>
T random(std::mt19937* generator, T lo, T hi);

template <>
inline int random(std::mt19937* generator, int lo, int hi) {
    std::uniform_int_distribution<> rand_distribution(lo, hi);
    return rand_distribution(*generator);
}

template <>
inline double random(std::mt19937* generator, double lo, double hi) {
    std::uniform_real_distribution<> rand_distribution(lo, hi);
    return rand_distribution(*generator);
}

/**
 * generate a random RobotInstant inside the field
 * @return random robot instant
 */
RobotInstant random_instant(std::mt19937* generator);

}  // namespace planning::TestingUtils