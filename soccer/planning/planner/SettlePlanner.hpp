#pragma once
#include "Planner.hpp"
#include "PathTargetPlanner.hpp"
namespace Planning {
class SettlePlanner: public PlannerForCommandType<SettleCommand> {
public:
    SettlePlanner(): PlannerForCommandType("SettleCommand") {}
    ~SettlePlanner() override = default;

    static void createConfiguration(Configuration* cfg);

    Trajectory plan(PlanRequest&& request);

private:
    RJ::Time bruteForceIntercept(const PlanRequest& request);
    Geometry2d::Pose findTargetPose(const PlanRequest& request);
    Trajectory intercept(PlanRequest&& request, RJ::Time interceptTime);

    // duration of the buffer before contacting the ball where the robot will
    // move at a constant velocity
    static ConfigDouble* _bufferTimeBeforeContact;

    // Gain on the averaging function to smooth the target point to intercept
    // This is due to the high flucations in the ball velocity frame to frame
    // a*newPoint + (1-a)*oldPoint
    // The lower the number, the less noise affects the system, but the slower
    // it responds to changes The higher the number, the more noise affects the
    // system, but the faster it responds to changes
    static ConfigDouble* _targetPointGain;

    // Closest dist to start searching for intercept points
    static ConfigDouble* _searchStartDist;  // m
    // Furthest dist to search for intercept points
    static ConfigDouble* _searchEndDist;  // m
    // What dist increment to search for intercepts
    static ConfigDouble* _searchIncDist;  // m

    // minimum distance the target needs to change before replanning
    static ConfigDouble* _targetChangeThreshold;

    // Distance between robot and closest point on ball line such that we move
    // directly into the ball line instead of trying to find the point we hit
    // first This does take into account slow moving balls in which we should
    // move onto the ball to capture it
    static ConfigDouble* _shortcutDist;

    std::array<std::optional<Geometry2d::Point>, Num_Shells> _avgTargetPoints;
    std::array<std::optional<Geometry2d::Point>, Num_Shells> _targetPoints;

    PathTargetPlanner _pathTargetPlanner;
};
}