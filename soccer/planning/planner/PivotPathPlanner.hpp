#pragma once

#include "Planner.hpp"

namespace Planning {
    
class PivotPathPlanner: public PlannerForCommandType<PivotCommand> {
public:
    static void createConfiguration(Configuration* cfg);
    Trajectory plan(PlanRequest&& request) override;
private:
    static std::unique_ptr<ConfigDouble> _pivotRadiusMultiplier;
};
}