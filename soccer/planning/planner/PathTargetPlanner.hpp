#pragma once

#include "LazyPlanner.hpp"
#include <vector>
#include "planning/trajectory/VelocityProfiling.hpp"

namespace Planning {

class PathTargetPlanner: public LazyPlanner {
public:
    PathTargetPlanner() = default;
    virtual ~PathTargetPlanner() = default;

    std::string name() const override { return "PathTargetPlanner"; }

    //todo(Ethan) write WorldVelPlanner
    bool isApplicable(const MotionCommand& command) const override {
        return std::holds_alternative<PathTargetCommand>(command);
    }
protected:
    RobotInstant getGoalInstant(const PlanRequest& request) const override {
        return std::get<PathTargetCommand>(request.motionCommand).pathGoal;
    }
};
} // namespace Planning
