#pragma once

#include "RRTPlanner.hpp"
#include "DirectTargetPathPlanner.hpp"
#include "SingleRobotPathPlanner.hpp"

class Configuration;
class ConfigDouble;

namespace Planning {

/**
 * @brief Planner which tries to move around the ball to intercept it
 *
 * TODO: Clean up description
 */
class SettlePathPlanner : public SingleRobotPathPlanner {
public:
    enum SettlePathPlannerStates {
        Intercept,
        Dampen,
        Complete
    };

    SettlePathPlanner() : SingleRobotPathPlanner(false), rrtPlanner(0, 250), directPlanner(),
                          interceptTarget(0,0), averageBallVel(0,0),
                          firstTargetPointFound(false),
                          firstBallVelFound(false),
                          pathCreatedForDampen(false),
                          currentState(Intercept) {};

    virtual std::unique_ptr<Path> run(PlanRequest& planRequest) override;

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::Settle;
    }

    static void createConfiguration(Configuration* cfg);

private:
    bool shouldReplan(const PlanRequest& planRequest) const;

    RRTPlanner rrtPlanner;
    DirectTargetPathPlanner directPlanner;
    boost::optional<Geometry2d::Point> targetFinalCaptureDirectionPos;

    SettlePathPlannerStates currentState;

    // Intercept Target Filtering Variables
    Geometry2d::Point interceptTarget;
    Geometry2d::Point averageBallVel;
    RJ::Seconds averagePathTime;
    bool firstTargetPointFound;
    bool firstBallVelFound;

    bool pathCreatedForDampen;

    // How much of the ball seed to contact the ball with
    // before slowing down to dampen the initial hit
    static ConfigDouble* _ballSpeedPercentForDampen; // %
    // Earliest time to start searching for intercept points
    static ConfigDouble* _searchStartTime; // Secs
    // Latest time to search for intercept points
    static ConfigDouble* _searchEndTime; // Secs
    // What increment of time to search for intercepts
    static ConfigDouble* _searchIncTime; // Secs
    // How much sooner should we reach the intercept point than we need to
    // Increase this to give us more time to reach the point to
    // compensate for bad motion control
    static ConfigDouble* _interceptBufferTime; // Sec
    // Gain on the averaging function to smooth the target point to intercept
    // This is due to the high flucations in the ball velocity frame to frame
    // a*newPoint + (1-a)*oldPoint
    // The lower the number, the less noise affects the system, but the slower it responds to changes
    // The higher the number, the more noise affects the system, but the faster it responds to changes
    static ConfigDouble* _targetPointGain;
    // Gain on the averaging function to smooth the ball velocity to for any motion commands
    // This is due to the high flucations in the ball velocity frame to frame
    // a*newPoint + (1-a)*oldPoint
    // The lower the number, the less noise affects the system, but the slower it responds to changes
    // The higher the number, the more noise affects the system, but the faster it responds to changes    
    static ConfigDouble* _ballVelGain;
    // Limits the max change in angle for the target intercept point
    // This is due to a bug in the velocity path planner
    // sometimes dropping speed significantly for a single frame
    // and messing up the intercept target
    static ConfigDouble* _maxAnglePathTargetChange; // Deg
    // If the ball velocity angle changes by a large amount
    // we want to quickly react and clear all the smoothing filters 
    static ConfigDouble* _maxBallAngleForReset; // Deg
};
}
