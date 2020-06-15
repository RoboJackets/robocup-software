#pragma once

#include <Geometry2d/Point.hpp>
#include <functional>
#include <optional>
#include <rrt/Tree.hpp>

#include "PathTargetPlanner.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/planner/Planner.hpp"

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

    Trajectory plan(const PlanRequest& planRequest) override {
        PlanRequest modified = planRequest;
        modified.motionCommand =
            PathTargetCommand{planRequest.start.linear_motion()};
        return _planner.plan(modified);
    }

    [[nodiscard]] bool isApplicable(
        const MotionCommand& /* command */) const override {
        return true;
    }

    /// Uses an RRT to find a point near to @pt that isn't blocked by obstacles.
    /// If @prevPt is give, only uses a newly-found point if it is closer to @pt
    /// by a configurable threshold.
    /// @param rrtLogger Optional callback to log the rrt tree after it's built
    static Geometry2d::Point findNonBlockedGoal(
        Geometry2d::Point pt, std::optional<Geometry2d::Point> prevPt,
        const Geometry2d::ShapeSet& obstacles, int maxItr = 300,
        std::function<void(const RRT::Tree<Geometry2d::Point>&)> rrtLogger =
            nullptr);

    static void createConfiguration(Configuration* cfg);

    static float stepSize() { return *_stepSize; }

private:
    PathTargetPlanner _planner;

    /// Step size for the RRT used to find an unblocked point in
    /// findNonBlockedGoal()
    static ConfigDouble* _stepSize;

    /// A newly-found unblocked goal must be this much closer to the start
    /// position than the previous point in order to be used.
    static ConfigDouble* _goalChangeThreshold;
};

}  // namespace Planning
