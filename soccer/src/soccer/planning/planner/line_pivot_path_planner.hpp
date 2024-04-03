#pragma once

#include "path_planner.hpp"
#include "path_target_path_planner.hpp"

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
class LinePivotPathPlanner : public PathPlanner {
public:
    LinePivotPathPlanner() : PathPlanner("line_pivot") {}
    ~LinePivotPathPlanner() override = default;

    LinePivotPathPlanner(LinePivotPathPlanner&&) noexcept = default;
    LinePivotPathPlanner& operator=(LinePivotPathPlanner&&) noexcept = default;
    LinePivotPathPlanner(const LinePivotPathPlanner&) = default;
    LinePivotPathPlanner& operator=(const LinePivotPathPlanner&) = default;

    Trajectory plan(const PlanRequest& request) override;

    void reset() override {
        cached_angle_change_ = std::nullopt;
        current_state_ = LINE;
    }
    [[nodiscard]] bool is_done() const override;

private:
    Trajectory previous_;

    // cache the most recent angle change so we know when we're done
    std::optional<double> cached_angle_change_;

    PathTargetPathPlanner path_target_{};

    Trajectory line(const PlanRequest& request);
    Trajectory pivot(const PlanRequest& request);

    // TODO(Kevin): ros param this
    double IS_DONE_ANGLE_CHANGE_THRESH = 1.0;

    enum State {
        LINE,
        PIVOT
    };

    State next_state(const PlanRequest& request);

    State current_state_ = LINE;
};
}  // namespace planning
