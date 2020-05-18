#pragma once
#include "PathTargetPlanner.hpp"
#include "Planner.hpp"
namespace Planning {
class LineKickPlanner : public PlannerForCommandType<LineKickCommand> {
public:
    LineKickPlanner()
        : PlannerForCommandType<LineKickCommand>("LineKickPlanner") {}
    ~LineKickPlanner() override = default;
    Trajectory plan(PlanRequest&& request) override;
    static void createConfiguration(Configuration* cfg);

private:
    enum class LineKickStates {
        Approach,
        FollowThrough
    };

    LineKickStates state;

    std::optional<Trajectory> attemptBruteForce(const PlanRequest& request);
    Trajectory planForSlowMovingBall(
        RobotInstant start, BallState ball, Geometry2d::Point target,
        Geometry2d::ShapeSet static_obstacles,
        const std::vector<DynamicObstacle>& dynamic_obstacles,
        RobotConstraints constraints);

    static ConfigDouble* _approachSpeed;

    Replanner _rrtPlanner;
    Trajectory previous;
};
}  // namespace Planning