#pragma once

#include "planning/planner/PathTargetPlanner.hpp"
#include "planning/planner/Planner.hpp"

namespace Planning {

/**
 * @brief Planner which captures a ball moving quickly towards us
 *
 * TODO: Clean up description
 */
class SettlePathPlanner : public PlannerForCommandType<SettleCommand> {
public:
    Trajectory plan(PlanRequest&& request) override;

    static Geometry2d::Point bruteForceGoal(const PlanRequest& request, Geometry2d::Point avgBallVel);

    static void createConfiguration(Configuration* cfg);

    std::string name() const override {
        return "SettlePathPlanner";
    }

    enum class SettleState {
        Intercept,
        Dampen
    };
private:
    Trajectory intercept(PlanRequest&& request);
    Trajectory dampen(PlanRequest&& request);

    void processStateTransitions(const Ball& ball, const OurRobot& robot, const RobotInstant& startInstant, SettleState& state);

    // Intercept Target Filtering Variables
    //todo(Ethan) delete averageBallVel?
    std::optional<Geometry2d::Point> averageBallVel;

    PathTargetPlanner pathTargetPlanner;
    //todo(Ethan) delete these static variables make them members, other planners too
    static std::vector<std::optional<Geometry2d::Point>> averageGoalPoints;
    static std::vector<SettleState> settleStates;
    static std::vector<RJ::Time> planTimes;

    // How much of the ball seed to contact the ball with
    // before slowing down to dampen the initial hit
    static ConfigDouble* _ballSpeedPercentForDampen;  // %

    // Closest dist to start searching for intercept points
    static ConfigDouble* _searchStartDist;  // m
    // Furthest dist to search for intercept points
    static ConfigDouble* _searchEndDist;  // m
    // What dist increment to search for intercepts
    static ConfigDouble* _searchIncDist;  // m

    // How much sooner should we reach the intercept point than we need to
    // This is a percent of the calculated intercept time
    // Numbers greater than 1 mean we increase intercept time needed by X% over
    // actual Numbers less than 1 mean we get there X% faster than we plan
    // (Shouldn't ever happen)
    static ConfigDouble* _interceptBufferTime;  // %

    // Gain on the averaging function to smooth the target point to intercept
    // This is due to the high flucations in the ball velocity frame to frame
    // a*newPoint + (1-a)*oldPoint
    // The lower the number, the less noise affects the system, but the slower
    // it responds to changes The higher the number, the more noise affects the
    // system, but the faster it responds to changes
    static ConfigDouble* _targetPointGain;

    // Gain on the averaging function to smooth the ball velocity to for any
    // motion commands This is due to the high flucations in the ball velocity
    // frame to frame a*newPoint + (1-a)*oldPoint The lower the number, the less
    // noise affects the system, but the slower it responds to changes The
    // higher the number, the more noise affects the system, but the faster it
    // responds to changes
    static ConfigDouble* _ballVelGain;

    // Distance between robot and closest point on ball line such that we move
    // directly into the ball line instead of trying to find the point we hit
    // first This does take into account slow moving balls in which we should
    // move onto the ball to capture it
    static ConfigDouble* _shortcutDist;  // m

    // If the ball velocity angle changes by a large amount
    // we want to quickly react and clear all the smoothing filters
    // Lower numbers means it reacts faster, but more chance for false positives
    // Higher numbers means slower reaction, but less false positives
    static ConfigDouble* _maxBallAngleForReset;  // Deg

    // If the ball velocity itself changes by a large amount
    // we want to quickly react and clear all the smoothing filters
    // Lower numbers means it reacts faster, but more chance for false positives
    // Higher numbers means slower reaction, but less false positives
    static ConfigDouble* _maxBallVelForPathReset;  // m/s

    // Max angle between ball and target bounce direction
    static ConfigDouble* _maxBounceAngle;  // Deg
};
}  // namespace Planning
