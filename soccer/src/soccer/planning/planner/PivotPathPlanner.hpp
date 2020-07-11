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

    void reset() override {
        previous = Trajectory{};
        cached_pivot_point = std::nullopt;
    }

private:
    Trajectory previous;

    // Cache the pivot point so we don't just push the ball across the field.
    std::optional<Geometry2d::Point> cached_pivot_point;

    static ConfigDouble* _pivotRadiusMultiplier;
};
}  // namespace Planning