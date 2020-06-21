#pragma once

#include "Planner.hpp"

namespace Planning {
class PivotPathPlanner : public PlannerForCommandType<PivotCommand> {
public:
    PivotPathPlanner()
        : PlannerForCommandType<PivotCommand>("PivotPathPlanner") {}
    ~PivotPathPlanner() override = default;

    PivotPathPlanner(PivotPathPlanner&&) noexcept = default;
    PivotPathPlanner& operator=(PivotPathPlanner&&) noexcept = default;
    PivotPathPlanner(const PivotPathPlanner&) = default;
    PivotPathPlanner& operator=(const PivotPathPlanner&) = default;

    static void createConfiguration(Configuration* cfg);
    Trajectory plan(const PlanRequest& request) override;

    void reset() override { previous = Trajectory{}; }

private:
    [[nodiscard]] bool shouldReplan(const PivotCommand& command) const;
    Trajectory previous;
    static ConfigDouble* _pivotRadiusMultiplier;
};
}  // namespace Planning