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
    bool is_done() const override;

private:
    Trajectory previous_;

    // Cache the pivot point and target so we don't just push the ball across the field.
    std::optional<rj_geometry::Point> cached_pivot_point_;
    std::optional<rj_geometry::Point> cached_pivot_target_;

    // cache the most recent angle change so we know when we're done
    std::optional<double> cached_angle_change_;

    // TODO(Kevin): ros param this
    // in gameplay it is 0.05, this is just to see the change
    double IS_DONE_ANGLE_CHANGE_THRESH = 0.1;
};
}  // namespace planning
