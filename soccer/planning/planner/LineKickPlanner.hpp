#pragma once

#include "Planner.hpp"
#include "planning/Instant.hpp"
#include "planning/primitives/Replanner.hpp"

namespace Planning {

/**
 * @brief Planner to kick a (possibly moving) ball without stopping to pivot
 * around it. The robot will move "through" the ball, and kick just as it comes
 * into contact with it.
 */
class LineKickPlanner : public PlannerForCommandType<LineKickCommand> {
public:
    LineKickPlanner()
        : PlannerForCommandType<LineKickCommand>("LineKickPlanner") {}
    ~LineKickPlanner() override = default;

    LineKickPlanner(LineKickPlanner&&) noexcept = default;
    LineKickPlanner& operator=(LineKickPlanner&&) noexcept = default;
    LineKickPlanner(const LineKickPlanner&) = default;
    LineKickPlanner& operator=(const LineKickPlanner&) = default;

    Trajectory plan(const PlanRequest& request) override;
    static void createConfiguration(Configuration* cfg);

private:
    enum class LineKickStates { Approach, FollowThrough };

    LineKickStates state = LineKickStates::Approach;

    std::optional<Trajectory> attemptBruteForce(const PlanRequest& request);
    Trajectory planForSlowMovingBall(
        RobotInstant start, BallState ball, Geometry2d::Point target,
        const Geometry2d::ShapeSet& static_obstacles,
        const std::vector<DynamicObstacle>& dynamic_obstacles,
        RobotConstraints constraints);

    static ConfigDouble* _approachSpeed;

    Replanner replanner;
    Trajectory previous;
};

}  // namespace Planning