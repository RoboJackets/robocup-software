#pragma once

#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include <rj_common/planning/instant.hpp>
#include <rj_common/planning/trajectory.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_geometry/shape_set.hpp>

#include "rj_planning/plan_request.hpp"
#include "rj_planning/planners/path_planner.hpp"
#include "rj_planning/primitives/replanner.hpp"
#include "rj_planning/primitives/velocity_profiling.hpp"

namespace planning {

/**
 * Planner identical to path_target but with no obstacle insertion.
 * Intended for controlled tests where obstacle avoidance should be disabled.
 */
class PathTargetTestPathPlanner : public PathPlanner {
public:
    PathTargetTestPathPlanner() : PathPlanner("path_target_test") {}
    ~PathTargetTestPathPlanner() override = default;

    PathTargetTestPathPlanner(PathTargetTestPathPlanner&&) noexcept = default;
    PathTargetTestPathPlanner& operator=(PathTargetTestPathPlanner&&) noexcept = default;
    PathTargetTestPathPlanner(const PathTargetTestPathPlanner&) = default;
    PathTargetTestPathPlanner& operator=(const PathTargetTestPathPlanner&) = default;

    Trajectory plan(const PlanRequest& request) override;
    void reset() override { previous_ = Trajectory(); }

    [[nodiscard]] bool is_done() const override;

private:
    [[nodiscard]] static AngleFunction get_angle_function(const PlanRequest& request);

    Trajectory previous_;

    // Vars to determine completion.
    std::optional<LinearMotionInstant> cached_start_instant_;
    std::optional<LinearMotionInstant> cached_target_instant_;
};

}  // namespace planning
