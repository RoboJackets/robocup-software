#pragma once

#include "SingleRobotPathPlanner.hpp"
#include "TrapezoidalPath.hpp"

namespace Planning {

/// Simple path planner that generates a straight-line path from the start
/// instant to the goal, ignoring all obstacles.  This class will eventually be
/// replaced by something better.
class DirectTargetPathPlanner : public Planner {
public:
    virtual std::unique_ptr<Path> run(PlanRequest& planRequest) override;
};

}  // namespace Planning
