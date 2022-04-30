#pragma once

#include <rj_geometry/point.hpp>

#include "trajectory.hpp"

namespace planning {

// TODO(#1501): both of these functions should be able to be abstracted under a
//  single interface templated on either a shape set or a collection of dynamic
//  obstacles.

/**
 * @brief Whether the given trajectory intersects any of the static obstacles
 *  given. Return this, along with the time and position of the first collision
 *  ("hit").
 *
 * @param trajectory   The trajectory to check.
 * @param obstacles    A set of obstacles to check against.
 * @param start_time   A start time to the entire check.
 * @param out_hit_time The time of the collision (output parameter).
 * @param out_hit_pt   The first point of contact of the collision (output param).
 * @return             Whether or not there is a collision.
 */
bool trajectory_hits_static(const Trajectory& trajectory, const rj_geometry::ShapeSet& obstacles,
                            RJ::Time start_time, RJ::Time* out_hit_time = nullptr,
                            rj_geometry::Point* out_hit_pt = nullptr);

/**
 * @brief Whether the given trajectory intersects any of the dynamic obstacles
 *  at any point along its path after a specified starting time.
 *
 * @param trajectory       The trajectory to check.
 * @param obstacles        A set of obstacles to check against.
 * @param start_time       A start time to the entire check.
 * @param out_hit_obstacle A bounding circle for the obstacle that was hit.
 * @param out_hit_time     The time of the collision (output parameter).
 * @return                 Whether or not there is a collision.
 */
bool trajectory_hits_dynamic(const Trajectory& trajectory,
                           const std::vector<DynamicObstacle>& obstacles,
                           RJ::Time start_time,
                           rj_geometry::Circle* out_hit_obstacle,
                           RJ::Time* out_hit_time);

}  // namespace planning
