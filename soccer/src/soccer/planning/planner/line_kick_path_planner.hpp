#pragma once

#include <optional>

#include "planning/planner/path_planner.hpp"
#include "planning/trajectory.hpp"

class Configuration;
class ConfigDouble;

namespace planning {

/**
 * PathPlanner which plans a path to line kick a ball.
 * Uses the System State object to get the position of the ball
 * and predict its Path. It chooses the closest intersection point
 * with the ball Path it can reach in time and plans a Path so the
 * ball and robot intersect at the same time.
 *
 * TODO(Kyle): Overhaul this entire planner. It's sketchy right now.
 *
 * Params taken from MotionCommand:
 *   target.position - planner will kick to this point
 */
class LineKickPathPlanner : public PathPlanner {
public:
    LineKickPathPlanner() : PathPlanner("line_kick"){};
    Trajectory plan(const PlanRequest& plan_request) override;

    void reset() override {
        prev_path_ = {};
        target_kick_pos_ = std::nullopt;
        latest_state_ = IDLING;
        average_ball_vel_initialized_ = false;
    }
    [[nodiscard]] bool is_done() const override;

private:
    State latest_state_ = IDLING;
    enum State {
        IDLING, INITIAL_APPROACH, FINAL_APPROACH
    };
    Trajectory prev_path_;
    std::optional<rj_geometry::Point> target_kick_pos_;
    bool average_ball_vel_initialized_ = false;
};

}  // namespace planning
