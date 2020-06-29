#pragma once

#include <optional>

#include "planning/Trajectory.hpp"
#include "planning/planner/Planner.hpp"

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
    Trajectory plan(const PlanRequest& planRequest) override;

    void reset() override {
        prevPath = {};
        finalApproach = false;
        targetKickPos = std::nullopt;
        reusePathCount = 0;
    }

private:
    Trajectory prevPath;
    bool finalApproach = false;
    std::optional<Geometry2d::Point> targetKickPos;
    int reusePathCount = 0;
};

}  // namespace Planning
