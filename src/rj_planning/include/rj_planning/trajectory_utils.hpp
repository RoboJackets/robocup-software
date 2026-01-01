#pragma once

#include <spdlog/spdlog.h>

#include <rj_common/planning/trajectory.hpp>
#include <rj_constants/constants.hpp>
#include <rj_planning/obstacle_set.hpp>

namespace planning {

/**
 * @brief Whether the given trajectory intersects any of the obstacles at
 *  any point along its path after a specified starting time.
 *
 * @param trajectory The trajectory to check.
 * @param obstacles  A set of obstacles to check against.
 * @param start_time A start time to the entire check.
 * @param hit_time   The time of the collision (output parameter).
 * @return           Whether or not there is a collision.
 */
bool trajectory_hits_static(const Trajectory& trajectory,
                          const ObstacleSet& obstacles,
                          RJ::Time start_time, RJ::Time* hit_time);

}  // namespace planning
