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
        // Moves to the ball path in front of it
        Intercept,
        // Starts to dampen the ball with backward motion
        Dampen
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

    // Restarts the state machine if our calculations are whack
    // and won't intercept ball correctly anymore
    void checkSolutionValidity(const Ball& ball, const MotionInstant& startInstant);

    // Figures out when to move to each state
    // (only in the standard transition)
    //
    // Note: startInstant may be changed inside this function
    //       when we are basically at the correct location and
    //       need to start the dampen
    void processStateTransition(const Ball& ball,
                                Path* prevPath,
                                const RJ::Seconds& timeIntoPreviousPath,
                                MotionInstant& startInstant);

    // State functions
    std::unique_ptr<Path> intercept(const PlanRequest& planRequest,
                                    const RJ::Time curTime,
                                    const MotionInstant& startInstant,
                                    std::unique_ptr<Path> partialPath,
                                    const RJ::Seconds partialPathTime,
                                    std::unique_ptr<Path> prevPath,
                                    const Geometry2d::ShapeSet& obstacles);

    std::unique_ptr<Path> dampen(const PlanRequest& planRequest,
                                 MotionInstant& startInstant,
                                 std::unique_ptr<Path> prevPath);

    std::unique_ptr<Path> invalid(const PlanRequest& planRequest);

    template<typename T>
    static T applyLowPassFilter(const T& oldValue, const T& newValue, double gain);

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
    int numInvalidPaths = 0;

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

    // How large of circle around ball to avoid
    static ConfigDouble* _ballAvoidDistance; // m
    
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
    
    // Limits the max number of invalid paths in a row
    // Specifically, an invalid path is a path where the velocity of
    // the robot is not continuous when the target state is not reacable
    // given the current acceleration constraints
    static ConfigInt* _maxNumInvalidPaths; // integer num
    
    // If the ball velocity angle changes by a large amount
    // we want to quickly react and clear all the smoothing filters 
    // Lower numbers means it reacts faster, but more chance for false positives
    // Higher numbers means slower reaction, but less false positives
    static ConfigDouble* _maxBallAngleForReset; // Deg

    // If the ball velocity itself changes by a large amount
    // we want to quickly react and clear all the smoothing filters
    // Lower numbers means it reacts faster, but more chance for false positives
    // Higher numbers means slower reaction, but less false positives
    static ConfigDouble* _maxBallVelForPathReset; // m/s
};
}
