#pragma once

#include "planning/planner/Planner.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/planner/MotionCommand.hpp"

namespace Planning {
//todo(Ethan) delete this planner, use PathTargetPlanner instead? should work right?
/// Simple path planner that generates a straight-line path from the start
/// instant to the goal, ignoring all obstacles.  This class will eventually be
/// replaced by something better.
class DirectTargetPathPlanner : public PlannerForCommandType<DirectPathTargetCommand> {
public:
    virtual Trajectory plan(PlanRequest&& planRequest) override;
    //todo(Ethan) delete this
    std::optional<RJ::Seconds> findInvalidTime(const PlanRequest& request) const;
    std::string name() const override { return "DirectTargetPathPlanner";}
};

}  // namespace Planning
