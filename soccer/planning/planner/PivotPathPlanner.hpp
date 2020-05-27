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
    bool shouldReplan(const PivotCommand& command) const;
    Trajectory previous;
    static ConfigDouble* _pivotRadiusMultiplier;
};
}  // namespace Planning