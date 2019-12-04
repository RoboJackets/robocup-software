#pragma once

#include "Planner.hpp"
#include "planning/trajectory/RoboCupStateSpace.hpp"
#include <Geometry2d/Point.hpp>
#include <vector>
#include <memory>

namespace Planning {

class PathTargetPlanner : public PlannerForCommandType<PathTargetCommand> {
public:
    PathTargetPlanner(): counter(0) {}

    Trajectory plan(PlanRequest&& request) override;

    std::string name() const override { return "PathTargetPlanner"; }
    static void createConfiguration(Configuration* cfg);

    static RJ::Seconds getPartialReplanLeadTime() { return RJ::Seconds(*_partialReplanLeadTime);}
private:
    bool goalChanged(const RobotInstant& prevGoal, const RobotInstant& goal) const;
    Trajectory planWithoutAngles(PlanRequest&& request);

    static ConfigDouble* _partialReplanLeadTime;

    int counter;
};
} // namespace Planning
