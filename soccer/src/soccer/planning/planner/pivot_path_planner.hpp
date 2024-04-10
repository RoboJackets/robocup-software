#pragma once

#include "path_planner.hpp"

namespace planning {

/**
 * PathPlanner which pivots about the point given by <command.pivot_point> to the target point
 * <command.target.position>.
 *
 * For instance, if <command.pivot_point> = ball.position and the ball is in
 * the robot's mouth, then the robot will pivot while maintaining contact with
 * the ball to <command.target.position>.
 *
 * Params taken from MotionCommand:
 *   target.pivot_point - robot will pivot about this point
 *   target.position - robot will face this point when done
 */
class PivotPathPlanner : public PathPlanner {
public:
    PivotPathPlanner() : PathPlanner("pivot") {}
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
        cached_angle_change_ = std::nullopt;
    }
    [[nodiscard]] bool is_done() const override;

private:
    Trajectory previous_;

    // Cache the pivot point and target so we don't just push the ball across the field.
    std::optional<rj_geometry::Point> cached_pivot_point_;
    std::optional<rj_geometry::Point> cached_pivot_target_;

    // cache the most recent angle change so we know when we're done
    std::optional<double> cached_angle_change_;

    // TODO(Kevin): ros param this
    static constexpr double IS_DONE_ANGLE_CHANGE_THRESH {1.0};
};
}  // namespace planning
