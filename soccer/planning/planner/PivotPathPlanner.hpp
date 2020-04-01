#pragma once

#include "Planner.hpp"

namespace Planning {
class PivotPathPlanner : public PlannerForCommandType<PivotCommand> {
public:
    PivotPathPlanner()
        : PlannerForCommandType<PivotCommand>("PivotPathPlanner") {}
    ~PivotPathPlanner() override = default;

    static void createConfiguration(Configuration* cfg);
    Trajectory plan(PlanRequest&& request) override;

private:
    static ConfigDouble* _pivotRadiusMultiplier;
};
}  // namespace Planning