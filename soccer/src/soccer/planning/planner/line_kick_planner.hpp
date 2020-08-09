#pragma once

#include <optional>

#include "planning/trajectory.hpp"
#include "planning/planner/planner.hpp"

class Configuration;
class ConfigDouble;

namespace Planning {

/**
 * Planner which plans a path to line kick a ball.
 * Uses the System State object to get the position of the ball
 * and predict its Path. It chooses the closest intersection point
 * with the ball Path it can reach in time and plans a Path so the
 * ball and robot intersect at the same time.
 *
 * TODO(Kyle): Overhaul this entire planner. It's sketchy right now.
 */
class LineKickPlanner
    : public PlannerForCommandType<Planning::LineKickCommand> {
public:
    LineKickPlanner()
        : PlannerForCommandType<Planning::LineKickCommand>("LineKickPlanner"){};
    Trajectory plan(const PlanRequest& plan_request) override;

    void reset() override {
        prev_path_ = {};
        final_approach_ = false;
        target_kick_pos_ = std::nullopt;
        reuse_path_count_ = 0;
    }

private:
    Trajectory prev_path_;
    bool final_approach_ = false;
    std::optional<rj_geometry::Point> target_kick_pos_;
    int reuse_path_count_ = 0;
};

}  // namespace Planning
