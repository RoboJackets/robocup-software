#pragma once
#include <vector>

#include <rj_geometry/shape_set.hpp>

#include "planner.hpp"
#include "planning/instant.hpp"
#include "planning/primitives/replanner.hpp"
#include "planning/primitives/velocity_profiling.hpp"
#include "rj_geometry/pose.hpp"

namespace planning {

class PathTargetPlanner : public Planner {
public:
    PathTargetPlanner() : Planner("PathTargetPlanner") = default;
    ~PathTargetPlanner() override = default;

    PathTargetPlanner(PathTargetPlanner&&) noexcept = default;
    PathTargetPlanner& operator=(PathTargetPlanner&&) noexcept = default;
    PathTargetPlanner(const PathTargetPlanner&) = default;
    PathTargetPlanner& operator=(const PathTargetPlanner&) = default;

    Trajectory plan(const PlanRequest& request) override;
    void reset() override { previous_ = Trajectory(); }

    [[nodiscard]] bool is_done() const override;

private:
    /*
     * Get the right AngleFunction (for control) from the
     * PTMC-specific PathTargetFaceOption options listed in
     * motion_command.hpp.
     *
     * @param PlanRequest containing a PTMC
     * @return AngleFunction that corresponds to input
     * PathTargetFaceOption in given PlanRequest
     */
    [[nodiscard]] static AngleFunction get_angle_function(
        const PlanRequest& request);

    Trajectory previous_;

    // vars to tell if is_done
    std::optional<LinearMotionInstant> cached_start_instant_;
    std::optional<LinearMotionInstant> cached_goal_instant_;
};

}  // namespace planning
