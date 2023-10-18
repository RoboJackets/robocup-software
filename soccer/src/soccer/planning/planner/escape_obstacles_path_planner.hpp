#pragma once

#include <optional>                      // for optional, nullopt
#include <rj_geometry/point.hpp>         // for Point
#include <string>                        // for allocator, string
#include "planning/planning_params.hpp"  // for PARAM_step_size
#include "planning/trajectory.hpp"       // for Trajectory
namespace planning { struct PlanRequest; }
namespace rj_geometry { class ShapeSet; }

namespace planning {
/**
 * @brief This planner finds a path to quickly get out of an obstacle. If the
 * start point isn't in an obstacle, returns a path containing only the start
 * point.
 *
 * Params taken from MotionCommand:
 *   None
 */
class EscapeObstaclesPathPlanner : public PathPlanner {
public:
    // TODO(Kevin): think of better way to convey halted behavior than "EscapeObstaclesPathPlanner"
    // actually this is not HALT, control node does HALT for us
    // or maybe use PathTargetPlanner with no real velocity?
    EscapeObstaclesPathPlanner() : PathPlanner("halt"){};
    ~EscapeObstaclesPathPlanner() override = default;

    EscapeObstaclesPathPlanner(EscapeObstaclesPathPlanner&&) noexcept = default;
    EscapeObstaclesPathPlanner& operator=(
        EscapeObstaclesPathPlanner&&) noexcept = default;
    EscapeObstaclesPathPlanner(const EscapeObstaclesPathPlanner&) = default;
    EscapeObstaclesPathPlanner& operator=(const EscapeObstaclesPathPlanner&) =
        default;

    Trajectory plan(const PlanRequest& plan_request) override;

    /// Uses an RRT to find a point near to @pt that isn't blocked by obstacles.
    /// If @prev_pt is give, only uses a newly-found point if it is closer to @pt
    /// by a configurable threshold.
    /// @param rrt_logger Optional callback to log the rrt tree after it's built
    static rj_geometry::Point find_non_blocked_goal(
        rj_geometry::Point pt, std::optional<rj_geometry::Point> prev_pt,
        const rj_geometry::ShapeSet& obstacles, int max_itr = 300);

    static double step_size() { return escape::PARAM_step_size; }

    void reset() override { previous_target_ = std::nullopt; }
    [[nodiscard]] bool is_done() const override;

private:
    /* PathTargetPathPlanner planner_; */
    std::optional<rj_geometry::Point> previous_target_;
};

}  // namespace planning
