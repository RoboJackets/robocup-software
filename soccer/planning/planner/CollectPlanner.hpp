#pragma once

#include "planning/Instant.hpp"
#include "planning/low_level/Replanner.hpp"
#include "planning/planner/Planner.hpp"

class Configuration;
class ConfigDouble;

namespace Planning {

/**
 * @brief Planner that tries to move onto and gain control of a slow moving
 * ball.
 */
class CollectPlanner : public PlannerForCommandType<CollectCommand> {
public:
    enum CollectPathPlannerStates {
        // From start of subbehavior to the start of the slow part of the
        // approach
        CourseApproach,
        // From the slow part of the approach to the touching of the ball
        FineApproach,
        // From touching the ball to stopped with the ball in the mouth
        Control
    };

    CollectPlanner()
        : PlannerForCommandType<CollectCommand>("collect"),
          averageBallVel(0, 0),
          approachDirection(0, 0) {}

    Trajectory plan(const PlanRequest& planRequest) override;

    static void createConfiguration(Configuration* cfg);

    void reset() override;

private:
    // Restarts the state machine if our calculations are whack
    // and won't intercept ball correctly anymore
    void checkSolutionValidity(BallState ball, RobotInstant start);

    void processStateTransition(BallState ball, RobotInstant startInstant);

    Trajectory courseApproach(
        const PlanRequest& planRequest, RobotInstant start,
        const Geometry2d::ShapeSet& staticObstacles,
        const std::vector<DynamicObstacle>& dynamicObstacles);

    Trajectory fineApproach(
        const PlanRequest& planRequest, RobotInstant startInstant,
        const Geometry2d::ShapeSet& staticObstacles,
        const std::vector<DynamicObstacle>& dynamicObstacles);

    Trajectory control(const PlanRequest& planRequest, RobotInstant start,
                       const Trajectory& partialPath,
                       const Geometry2d::ShapeSet& staticObstacles,
                       const std::vector<DynamicObstacle>& dynamicObstacles);

    Trajectory invalid(const PlanRequest& planRequest,
                       const Geometry2d::ShapeSet& staticObstacles,
                       const std::vector<DynamicObstacle>& dynamicObstacles);

    Trajectory previous;

    CollectPathPlannerStates currentState =
        CollectPathPlannerStates::CourseApproach;

    // Ball Velocity Filtering Variables
    Geometry2d::Point averageBallVel;
    bool averageBallVelInitialized = false;

    Geometry2d::Point approachDirection;
    bool approachDirectionCreated = false;

    bool controlPathCreated = false;

    Geometry2d::Point pathCourseTarget;
    bool pathCoarseTargetInitialized = false;

    // Controls at which ball speed we should try to go directly to the ball
    // or to move behind it and in the same direction as it
    //
    // Low number indicates that it will always try to choose a point for the
    // robot behind the velocity vector
    //
    // High number indicates that it will always try to choose a point nearest
    // to the current robot position
    static ConfigDouble* _ballSpeedApproachDirectionCutoff;  // m/s

    // How much to scale the accelerations by as a percent of the normal
    // acceleration
    //
    // Approach acceleration controls all the movement from the start location
    // to right before touching the ball
    static ConfigDouble* _approachAccelScalePercent;  // %
    // Control acceleration controls the touch to stop
    // Lower this if we decelerate too quickly for the dribbler to keep a back
    // spin on
    static ConfigDouble* _controlAccelScalePercent;  // %

    // How far away from the ball to target for the approach
    // This should be tuned so that the end of approach places the dribbler
    // touching the ball Increase the distance so that when we overshoot, it
    // doesn't hit the ball Ideally this is 0
    static ConfigDouble* _approachDistTarget;  // m

    // At what speed should we be when we touch the ball (Well, the difference
    // in speed between the ball and robot) Should be as low as possible where
    // we still are able to touch the ball and control it If we are slamming
    // into the ball decrease this number If we aren't even touching it to the
    // dribbler, increase this number (And check the approachDistTarget)
    static ConfigDouble* _touchDeltaSpeed;  // m/s

    static ConfigDouble* _velocityControlScale;  // %

    // How close to the ball do we have to be before transferring to the control
    // state This should be almost zero Increase if the noise on vision causes
    // problems and we never transition
    static ConfigDouble* _distCutoffToControl;  // m

    // How close to the ball do we need to be before transferring back to the
    // approach state and restarting This should be just above zero to account
    // for noise in vision
    static ConfigDouble* _distCutoffToApproach;  // m

    // How close to the target velocity do we have to be before transferring to
    // the control state This doesn't matter as much right now It's good to be
    // close, but it's not too big of deal If we are trying to transition with a
    // large delta velocity between the robot and the ball decrease this number
    static ConfigDouble* _velCutoffToControl;  // m/s

    // How much extra room should we stay at the delta speed before slowing down
    // This is really a percent of the minimum stop distance to fully stop given
    // the current velocity This should be tuned such that we dont drive too far
    // through the ball A number of 1 should mean a constant acceleration
    // through the entire sequence Increasing this number makes the robot state
    // at delta velocity longer
    static ConfigDouble* _stopDistScale;  // %

    // Gain on the averaging function to smooth the target point to intercept
    // This is due to the high flucations in the ball velocity frame to frame
    // a*newPoint + (1-a)*oldPoint
    // The lower the number, the less noise affects the system, but the slower
    // it responds to changes The higher the number, the more noise affects the
    // system, but the faster it responds to changes
    static ConfigDouble* _targetPointAveragingGain;
};

}  // namespace Planning