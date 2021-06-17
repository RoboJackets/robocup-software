#pragma once

#include "planner.hpp"

namespace planning {
class PivotPathPlanner : public PlannerForCommandType<PivotCommand> {
public:
    PivotPathPlanner()
        : PlannerForCommandType<PivotCommand>("PivotPathPlanner") {}
    ~PivotPathPlanner() override = default;

    PivotPathPlanner(PivotPathPlanner&&) noexcept = default;
    PivotPathPlanner& operator=(PivotPathPlanner&&) noexcept = default;
    PivotPathPlanner(const PivotPathPlanner&) = default;
    PivotPathPlanner& operator=(const PivotPathPlanner&) = default;

    Trajectory plan(const PlanRequest& request) override;

    void reset() override {
        previous_ = Trajectory{};
        cached_pivot_target_ = std::nullopt;
        cached_pivot_point_ = std::nullopt;
    }

private:
    Trajectory previous_;

    // Cache the pivot point and target so we don't just push the ball across the field.
    std::optional<rj_geometry::Point> cached_pivot_point_;
    std::optional<rj_geometry::Point> cached_pivot_target_;
};
}  // namespace planning