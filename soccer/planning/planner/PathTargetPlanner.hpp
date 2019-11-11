#pragma once

#include "Planner.hpp"

namespace Planning {

class PathTargetPlanner : public PlannerForCommandType<PathTargetCommand> {
public:
    Trajectory plan(PlanRequest&& request) override;

    std::string name() const override { return "PathTargetPlanner"; }

    static void createConfiguration(Configuration* cfg);

protected:
    static ConfigDouble* _partialReplanLeadTime;
};

} // namespace Planning
