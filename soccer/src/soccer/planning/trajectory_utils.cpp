#include "trajectory_utils.hpp"

#include <rj_constants/constants.hpp>
#include <rj_geometry/point.hpp>

namespace planning {

// TODO: if this works, change the comment in corresponding .hpp
// TODO: also change the dynamic version below
std::vector<rj_geometry::Point> trajectory_hits_static(const Trajectory& trajectory,
                                                       const rj_geometry::ShapeSet& obstacles,
                                                       RJ::Time start_time, RJ::Time* hit_time) {
    // construct path_breaks, a list of points where the straight path must be
    // broken to account for an obstacle
    // (list will solely contain points outside obstacles)
    std::vector<rj_geometry::Point> path_breaks;

    if (trajectory.empty()) {
        return path_breaks;
    }

    if (start_time < trajectory.begin_time()) {
        throw std::invalid_argument(
            "Error in Trajectory::hit(): Start time cannot "
            "be before trajectory begin");
    }

    Trajectory::Cursor cursor = trajectory.cursor(start_time);

    // If the trajectory has already ended, we don't need to check it.
    if (!cursor.has_value()) {
        return path_breaks;
    }

    // Limit iterations to 100. This will continue to operate at dt = 0.05 until
    // we hit a 5 second trajectory. If our trajectory is longer than that,
    // something is probably wrong, but we'll still handle it (just scale dt
    // accordingly).
    // TODO(#1525): Make these config variables.
    constexpr int kMaxIterations = 100;
    constexpr RJ::Seconds kExpectedDt{0.05};

    RJ::Seconds time_left{trajectory.end_time() - start_time};
    RJ::Seconds dt = std::max(kExpectedDt, time_left / kMaxIterations);

    // set of obstacles that hit the cursor's start position
    // (cursor is an iterator over the trajectory)
    const auto& start_hits = obstacles.hit_set(cursor.value().position());

    // last hit obstacle on cursor's path
    rj_geometry::Point last_pos;
    bool last_pt_was_hit = false;

    while (cursor.has_value()) {
        RobotInstant instant = cursor.value();
        auto position = instant.position();

        // find all collisions along path
        bool hit_found = false;
        for (const auto& obstacle : obstacles.shapes()) {
            if (obstacle->hit(position) && start_hits.find(obstacle) == start_hits.end()) {
                // Only count hits that we didn't start in.

                // save hit_time if ptr given
                // (is given nowhere as far as I can tell -Kevin)
                if (hit_time != nullptr) {
                    *hit_time = instant.stamp;
                }

                // mark hit found if found
                // (hit is at cursor's current position)
                hit_found = true;
            }
        }

        // save the points just outside obstacles on the line by
        // detecting transition from hit to not hit (or vice versa)
        if (hit_found && !last_pt_was_hit) {
            // this is first pos inside a new obstacle, so save last pos outside it
            path_breaks.push_back(last_pos);
        } else if (!hit_found && last_pt_was_hit) {
            // this is first pos outside the obstacle, so save this pos
            path_breaks.push_back(position);
        }

        // update last_pos, last_pt_was_hit, cursor
        last_pos = position;
        if (hit_found) {
            last_pt_was_hit = true;
        }
        cursor.advance(dt);
    }

    return path_breaks;
}

bool trajectory_hits_dynamic(const Trajectory& trajectory,
                             const std::vector<DynamicObstacle>& obstacles, RJ::Time start_time,
                             rj_geometry::Circle* out_hit_obstacle, RJ::Time* out_hit_time) {
    if (trajectory.empty()) {
        return false;
    }

    if (start_time < trajectory.begin_time()) {
        throw std::invalid_argument(
            "Error in Trajectory::hit(): Start time cannot "
            "be before trajectory begin");
    }

    Trajectory::Cursor cursor = trajectory.cursor(start_time);

    // If the trajectory has already ended, we don't need to check it.
    if (!cursor.has_value()) {
        return false;
    }

    // Limit iterations to 100. This will continue to operate at dt = 0.05 until
    // we hit a 5 second trajectory. If our trajectory is longer than that,
    // something is probably wrong, but we'll still handle it (just scale dt
    // accordingly).
    // TODO(#1525): Make these config variables.
    constexpr int kMaxIterations = 100;
    constexpr RJ::Seconds kExpectedDt{0.05};

    RJ::Seconds time_left{trajectory.end_time() - start_time};
    RJ::Seconds dt = std::max(kExpectedDt, time_left / kMaxIterations);

    // The time of the earliest hit, if there is one. This is needed so that
    // we get the _first_ time we hit an obstacle, not necessarily the time we
    // hit the obstacle that happened to be first in the list.
    std::optional<RJ::Time> maybe_hit_time = std::nullopt;

    for (const DynamicObstacle& obs : obstacles) {
        if (obs.path->empty()) {
            throw std::runtime_error("Empty trajectory in dynamic obstacle");
        }

        cursor.seek(start_time);

        // Inflate obstacles by our robot's radius.
        const double total_radius = obs.circle.radius() + kRobotRadius;

        // Only use the trajectory cursor in the loop condition; we use the
        // static position after the obstacle cursor runs off the end.
        for (auto cursor_obstacle = obs.path->cursor_begin(); cursor.has_value();
             cursor_obstacle.advance(dt), cursor.advance(dt)) {
            // If the earlier calculated hit was before this point, stop looking
            // at this obstacle.
            if (maybe_hit_time.has_value() && maybe_hit_time.value() < cursor.time()) {
                break;
            }

            rj_geometry::Point obstacle_position;
            if (cursor_obstacle.has_value()) {
                obstacle_position = cursor_obstacle.value().position();
            } else {
                obstacle_position = obs.path->last().position();
            }

            rj_geometry::Point robot_position = cursor.value().position();

            if (robot_position.dist_to(obstacle_position) < total_radius) {
                // We would already have broken out if we had an earlier
                // obstacle (from the check above), so this is definitely the
                // earliest one.
                if (out_hit_obstacle != nullptr) {
                    *out_hit_obstacle = rj_geometry::Circle(obstacle_position, obs.circle.radius());
                }
                maybe_hit_time = cursor.time();
            }
        }
    }

    if (maybe_hit_time.has_value() && out_hit_time != nullptr) {
        *out_hit_time = maybe_hit_time.value();
    }

    return maybe_hit_time.has_value();
}

}  // namespace planning
