#pragma once

#include <memory>
#include <vector>

#include <rj_common/planning/instant.hpp>
#include <rj_common/planning/trajectory.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_geometry/util.hpp>
#include <rj_param_utils/planning/planning_params.hpp>

#include "rj_planning/planners/path_planner.hpp"
#include "rj_planning/planners/path_target_path_planner.hpp"
#include "rj_planning/primitives/angle_planning.hpp"
#include "rj_planning/primitives/path_smoothing.hpp"
#include "rj_planning/primitives/trapezoidal_motion.hpp"
#include "rj_planning/primitives/velocity_profiling.hpp"

namespace planning {
/**
 * Path planner that only rotates the robot about a point given.
 *
 * Params taken from MotionCommand:
 *  target.position - robot will face this point when done
 *  target.pivot_point - robot will pivot around this point
 */
class RotatePathPlanner : public PathPlanner {
public:
    RotatePathPlanner() : PathPlanner("rotate") {}
    ~RotatePathPlanner() override = default;

    RotatePathPlanner(RotatePathPlanner&&) noexcept = default;
    RotatePathPlanner& operator=(RotatePathPlanner&&) noexcept = default;
    RotatePathPlanner(const RotatePathPlanner&) = default;
    RotatePathPlanner& operator=(const RotatePathPlanner&) = default;

    Trajectory plan(const PlanRequest& request) override;

    void reset() override {
        cached_target_angle_ = std::nullopt;
        cached_angle_change_ = std::nullopt;
        current_state_ = PIVOT;
    }
    [[nodiscard]] bool is_done() const override;

private:
    Trajectory previous_;

    std::optional<double> cached_target_angle_;  // equivalent to previously recorded accorded
    std::optional<double> cached_angle_change_;

    std::optional<Trajectory> cached_path_;

    PathTargetPathPlanner path_target_{};

    Trajectory pivot(const PlanRequest& request);
    Trajectory end(const PlanRequest& request);
    void update_state();

    enum State { PIVOT, END };
    RotatePathPlanner::State current_state_ = RotatePathPlanner::State::PIVOT;

    double isDoneAngleChangeThresh{1.0};
};
}  // namespace planning