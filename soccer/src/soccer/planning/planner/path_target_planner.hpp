#pragma once
#include <vector>

#include <rj_geometry/shape_set.hpp>

#include "planner.hpp"
#include "planning/instant.hpp"
#include "planning/primitives/replanner.hpp"
#include "planning/primitives/velocity_profiling.hpp"
#include "rj_geometry/pose.hpp"

namespace planning {

/**
 * Planner which moves the robot to a desired point at a desired velocity. Will
 * avoid all obstacles given in plan_request. Will avoid ball unless
 * ignore_ball = true.
 *
 * Params taken from MotionCommand:
 *   target.position - robot will end up at this point
 *   target.velocity - robot will end up with this velocity on target.position
 *   face_option - while travelling, robot will face the target point unless
 *                 this is specified (see motion_command.hpp)
 *
 *   ignore_ball - when false, robot treats ball as obstacle
 */
class PathTargetPlanner : public Planner {
public:
    PathTargetPlanner() : Planner("path_target") {}
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
    std::optional<LinearMotionInstant> cached_target_instant_;
};

}  // namespace planning
