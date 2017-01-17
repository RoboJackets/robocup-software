#pragma once

#include <Geometry2d/Point.hpp>
#include "RRTPlanner.hpp"
#include "SingleRobotPathPlanner.hpp"
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
 * TODO(ashaw596): Fix bug with replanning on real robots.
 */
class LineKickPlanner : public SingleRobotPathPlanner {
public:
    LineKickPlanner() : SingleRobotPathPlanner(false), rrtPlanner(250){};
    virtual std::unique_ptr<Path> run(PlanRequest& planRequest) override;

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::LineKick;
    }

private:
    bool shouldReplan(const PlanRequest& planRequest) const;

    RRTPlanner rrtPlanner;
    bool finalApproach = false;
    boost::optional<Geometry2d::Point> targetKickPos;
};

}  // namespace Planning
