#pragma once

#include <memory>
#include <vector>

#include <rj_constants/constants.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_geometry/util.hpp>
#include <rj_common/planning/instant.hpp>
#include <rj_common/planning/trajectory.hpp>
#include <rj_param_utils/planning/planning_params.hpp>

#include "rj_planning/planners/path_planner.hpp"
#include "rj_planning/primitives/angle_planning.hpp"
#include "rj_planning/primitives/path_smoothing.hpp"
#include "rj_planning/primitives/trapezoidal_motion.hpp"
#include "rj_planning/primitives/velocity_profiling.hpp"

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
    static constexpr double kIsDoneAngleChangeThresh{1.0};
};
}  // namespace planning
