#pragma once

#include <rj_geometry/point.hpp>
#include <functional>
#include <optional>
#include <rrt/Tree.hpp>

#include "path_target_planner.hpp"
#include "planning/planner/plan_request.hpp"
#include "planning/planner/planner.hpp"

class Configuration;
class ConfigDouble;

namespace Planning {
/**
 * @brief This planner finds a path to quickly get out of an obstacle. If the
 * start point isn't in an obstacle, returns a path containing only the start
 * point.
 */
class EscapeObstaclesPathPlanner : public Planner {
public:
    EscapeObstaclesPathPlanner() : Planner("EscapeObstaclesPathPlanner"){};
    ~EscapeObstaclesPathPlanner() override = default;

    EscapeObstaclesPathPlanner(EscapeObstaclesPathPlanner&&) noexcept = default;
    EscapeObstaclesPathPlanner& operator=(
        EscapeObstaclesPathPlanner&&) noexcept = default;
    EscapeObstaclesPathPlanner(const EscapeObstaclesPathPlanner&) = default;
    EscapeObstaclesPathPlanner& operator=(const EscapeObstaclesPathPlanner&) =
        default;

    Trajectory plan(const PlanRequest& plan_request) override;

    [[nodiscard]] bool is_applicable(
        const MotionCommand& /* command */) const override {
        return true;
    }

    /// Uses an RRT to find a point near to @pt that isn't blocked by obstacles.
    /// If @prev_pt is give, only uses a newly-found point if it is closer to @pt
    /// by a configurable threshold.
    /// @param rrt_logger Optional callback to log the rrt tree after it's built
    static rj_geometry::Point find_non_blocked_goal(
        rj_geometry::Point pt, std::optional<rj_geometry::Point> prev_pt,
        const rj_geometry::ShapeSet& obstacles, int max_itr = 300);

    static void create_configuration(Configuration* cfg);

    static double step_size() { return *step_size_config; }

private:
    PathTargetPlanner planner_;

    /// Step size for the RRT used to find an unblocked point in
    /// find_non_blocked_goal()
    static ConfigDouble* step_size_config;

    /// A newly-found unblocked goal must be this much closer to the start
    /// position than the previous point in order to be used.
    static ConfigDouble* goal_change_threshold;
};

}  // namespace Planning
