#pragma once

#include "SingleRobotPathPlanner.hpp"
#include "TrapezoidalPath.hpp"

namespace Planning {

/// Simple path planner that generates a straight-line path from the start
/// instant to the goal, ignoring all obstacles.  This class will eventually be
/// replaced by something better.
class DirectTargetPathPlanner : public SingleRobotPathPlanner {
public:
    DirectTargetPathPlanner() : SingleRobotPathPlanner(false){};
    MotionCommand::CommandType commandType() const override {
        return MotionCommand::CommandType::DirectPathTarget;
    }

    virtual std::unique_ptr<Path> run(PlanRequest& planRequest) override;

    bool shouldReplan(const PlanRequest& planRequest) const;
};

}  // namespace Planning
