#include "rj_planning/trajectory_utils.hpp"

namespace planning {

bool trajectory_hits_static(const Trajectory& trajectory, const ObstacleSet& obstacles,
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
        for (const auto& obstacle : obstacles.obstacles()) {
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

bool trajectory_hits_static_optimized(const Trajectory& trajectory, const ObstacleSet& obstacles,
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
    constexpr int kMaxIterations = 100;
    constexpr RJ::Seconds kExpectedDt{0.05};

    RJ::Seconds time_left{trajectory.end_time() - start_time};
    RJ::Seconds dt = std::max(kExpectedDt, time_left / kMaxIterations);

    // PHASE 1: Core-only collision check (fast rejection)
    // Check obstacle cores first - these are smaller shapes compared to
    // padding.
    Trajectory::Cursor core_cursor = trajectory.cursor(start_time);
    while (core_cursor.has_value()) {
        RobotInstant instant = core_cursor.value();

        // Check if we hit any obstacle CORE
        if (obstacles.obstacle_hit(instant.position())) {
            if (hit_time != nullptr) {
                *hit_time = instant.stamp;
            }
            return true;  // Hard collision - reject immediately
        }

        core_cursor.advance(dt);
    }

    // PHASE 2: Padding collision check with relaxed handling
    // Only runs if Phase 1 passed. This is the expensive check
    // due to large stadium shapes for moving obstacles. We use the same relaxed
    // logic as the original: allow starting in padding, but reject entering NEW padding.

    Trajectory::Cursor padding_cursor = trajectory.cursor(start_time);
    const auto& start_hits = obstacles.hit_set(padding_cursor.value().position());
    while (padding_cursor.has_value()) {
        RobotInstant instant = padding_cursor.value();

        // Only count hits that we didn't start in.
        for (const auto& obstacle : obstacles.obstacles()) {
            if (obstacle->padding_hit(instant.position()) &&
                start_hits.find(obstacle) == start_hits.end()) {
                if (hit_time != nullptr) {
                    *hit_time = instant.stamp;
                }
                return true;
            }
        }

        padding_cursor.advance(dt);
    }

    // No collisions detected - trajectory is valid
    return false;
}

}  // namespace planning
