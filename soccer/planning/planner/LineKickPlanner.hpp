#pragma once
#include "Planner.hpp"
#include "PathTargetPlanner.hpp"
namespace Planning{
class LineKickPlanner: public PlannerForCommandType<LineKickCommand> {
public:
    LineKickPlanner(): PlannerForCommandType<LineKickCommand>("LineKickPlanner") {}
    ~LineKickPlanner() override = default;
    Trajectory plan(PlanRequest&& request) override;
    static void createConfiguration(Configuration* cfg);
private:
    std::optional<Trajectory> attemptBruteForce(const PlanRequest& request);
    Trajectory planForSlowMovingBall(PlanRequest&& request);

    static ConfigDouble* _approachSpeed;

    PathTargetPlanner _pathTargetPlanner;
    bool _finalApproach = false;
    std::optional<Geometry2d::Point> _targetKickPos;
    int _reusePathCount = 0;
};
}