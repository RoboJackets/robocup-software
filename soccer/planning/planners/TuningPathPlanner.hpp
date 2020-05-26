#pragma once

#include "SingleRobotPathPlanner.hpp"
#include "planning/paths/TrapezoidalPath.hpp"

namespace Planning {

/// Simple path planner that breaks everything
class TuningPathPlanner : public SingleRobotPathPlanner {
public:
    TuningPathPlanner() : SingleRobotPathPlanner(false){};
    MotionCommand::CommandType commandType() const override {
        return MotionCommand::CommandType::TuningPath;
    }

    virtual std::unique_ptr<Path> run(PlanRequest& planRequest) override;

    bool shouldReplan(const PlanRequest& planRequest) const;
};

}  // namespace Planning
