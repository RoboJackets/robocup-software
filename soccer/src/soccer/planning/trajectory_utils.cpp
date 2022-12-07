#include "trajectory_utils.hpp"

#include <rj_constants/constants.hpp>

namespace planning {

bool trajectory_hits_static(const Trajectory& trajectory, const rj_geometry::ShapeSet& obstacles,
                            RJ::Time start_time, RJ::Time* hit_time) {
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

    const auto& start_hits = obstacles.hit_set(cursor.value().position());
    while (cursor.has_value()) {
        RobotInstant instant = cursor.value();

        // Only count hits that we didn't start in.
        for (const auto& obstacle : obstacles.shapes()) {
            if (obstacle->hit(instant.position()) &&
                start_hits.find(obstacle) == start_hits.end()) {
                if (hit_time != nullptr) {
                    *hit_time = instant.stamp;
                }
                return true;
            }
        }

        cursor.advance(dt);
    }

    // No obstacles were hit, and we're through the whole trajectory.
    return false;
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

    try {
        for (const DynamicObstacle& obs : obstacles) {
            if (obs.path->empty()) {
                throw std::runtime_error("Empty trajectory in dynamic obstacle");
            }

            // TODO: REMOVE THIS
            obs.path->cursor_begin();

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
                        *out_hit_obstacle =
                            rj_geometry::Circle(obstacle_position, obs.circle.radius());
                    }
                    maybe_hit_time = cursor.time();
                }
            }
        }
    } catch (...) {
        SPDLOG_ERROR("exception caught !!!");
    }

    // if there was a collision, send the timestamp back via out_hit_time and
    // return true (else return false and leave out_hit_time untouched)
    if (maybe_hit_time.has_value() && out_hit_time != nullptr) {
        *out_hit_time = maybe_hit_time.value();
    }
    return maybe_hit_time.has_value();
}

}  // namespace planning
