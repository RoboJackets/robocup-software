#include "CollectPathPlanner.hpp"

#include "CompositePath.hpp"
#include "MotionInstant.hpp"
#include "Configuration.hpp"
#include "Constants.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

REGISTER_CONFIGURABLE(CollectPathPlanner);

ConfigDouble* CollectPathPlanner::_ballSpeedApproachDirectionCutoff;
ConfigDouble* CollectPathPlanner::_approachAccelScalePercent;
ConfigDouble* CollectPathPlanner::_controlAccelScalePercent;
ConfigDouble* CollectPathPlanner::_approachDistTarget;
ConfigDouble* CollectPathPlanner::_touchDeltaSpeed;
ConfigDouble* CollectPathPlanner::_distCutoffToControl;
ConfigDouble* CollectPathPlanner::_velCutoffToControl;
ConfigDouble* CollectPathPlanner::_distCutoffToApproach;
ConfigDouble* CollectPathPlanner::_stopDistScale;
ConfigDouble* CollectPathPlanner::_targetPointAveragingGain;

void CollectPathPlanner::createConfiguration(Configuration* cfg) {
    _ballSpeedApproachDirectionCutoff =
        new ConfigDouble(cfg, "Capture/Collect/ballSpeedApproachDirectionCutoff", 0.1);
    _approachAccelScalePercent =
        new ConfigDouble(cfg, "Capture/Collect/approachAccelScalePercent", 0.7);
    _controlAccelScalePercent =
        new ConfigDouble(cfg, "Capture/Collect/controlAccelScalePercent", 0.8);
    _approachDistTarget =
        new ConfigDouble(cfg, "Capture/Collect/approachDistTarget", 0.04);
    _touchDeltaSpeed =
        new ConfigDouble(cfg, "Capture/Collect/touchDeltaSpeed", 0.1);
    _distCutoffToControl =
        new ConfigDouble(cfg, "Capture/Collect/distCutoffToControl", 0.05);
    _velCutoffToControl =
        new ConfigDouble(cfg, "Capture/Collect/velCutoffToControl", 1);
    _distCutoffToApproach =
        new ConfigDouble(cfg, "Capture/Collect/distCutoffToApproach", 0.3);
    _stopDistScale =
        new ConfigDouble(cfg, "Capture/Collect/stopDistScale", 1);
    _targetPointAveragingGain =
        new ConfigDouble(cfg, "Capture/Collect/targetPointAveragingGain", 0.8);
}

bool CollectPathPlanner::shouldReplan(const PlanRequest& planRequest) const {
    // TODO: Figure out when a replan is needed
    return true;
}

std::unique_ptr<Path> CollectPathPlanner::run(PlanRequest& planRequest) {
    SystemState& systemState = planRequest.systemState;
    const Ball& ball = systemState.ball;

    const RJ::Time curTime = RJ::now();

    const CollectCommand& command = dynamic_cast<const CollectCommand&>(*planRequest.motionCommand);

    // Start state for specified robot
    MotionInstant& startInstant = planRequest.start;
    // All the max velocity / acceleration constraints for translation / rotation
    RobotConstraints robotConstraints = planRequest.constraints;
    MotionConstraints& motionConstraints = robotConstraints.mot;

    // List of obstacles
    ShapeSet& obstacles = planRequest.obstacles;
    vector<DynamicObstacle>& dynamicObstacles = planRequest.dynamicObstacles;

    // Previous RRT path from last iteration
    std::unique_ptr<Path>& prevPath = planRequest.prevPath;

    // The small beginning part of the previous path
    unique_ptr<Path> partialPath = nullptr;

    // The is from the original robot position to the ball
    // We only care about the replan lead time from the current pos in the path
    // to the intercept point
    RJ::Seconds timeIntoPreviousPath;

    // How much of the future we are devoting to the partial path
    // 0ms unless we have a partial path, then it's a partialReplanLeadTime
    RJ::Seconds partialPathTime = 0ms;

    // How much of the previous path to steal
    const RJ::Seconds partialReplanLeadTime = RRTPlanner::getPartialReplanLeadTime();

    // Change start instant to be the partial path end instead of the robot current location
    // if we actually have already calculated a path the frame before
    if (prevPath) {
        timeIntoPreviousPath = curTime - prevPath->startTime();

        // Make sure we still have time in the path to replan and correct
        // since it's likely that the old path is slightly off
        //
        // ---|----------------|-----------------|
        // TimeNow     EndPartialPrevPath  FinalTargetPoint
        //                     |-----------------|
        //          Amount of the path we can change this iteration
        if (timeIntoPreviousPath < prevPath->getDuration() - 2*partialReplanLeadTime &&
            timeIntoPreviousPath > 0ms) {
            partialPath =
                prevPath->subPath(0ms, timeIntoPreviousPath + partialReplanLeadTime);
            partialPathTime = partialPath->getDuration() - timeIntoPreviousPath;
            startInstant = partialPath->end().motion;
        }
    }

    // Initialize the filter to the ball velocity so there's less ramp up
    if (!averageBallVelInitialized) {
        averageBallVel = ball.vel;
        averageBallVelInitialized = true;
    } else {
        averageBallVel = *_targetPointAveragingGain * averageBallVel + 
                         (1 - *_targetPointAveragingGain) * ball.vel;
    }

    // Do the transitions
    float dist = (startInstant.pos - ball.pos).mag() - Robot_MouthRadius - *_approachDistTarget;
    float speedDiff = (startInstant.vel - averageBallVel).mag() - *_touchDeltaSpeed;

    // If we are close enough to the target point near the ball
    // and almost the same speed we want, start slowing down
    // TODO: Check for ball sense?
    if (dist < *_distCutoffToControl && speedDiff < *_velCutoffToControl && currentState == Approach) {
        currentState = Control;
    }

    // Check if we need to go back into approach
    if ((ball.pos - startInstant.pos).mag() > 2*Robot_Radius && currentState == Control) {
        cout << "Reseting" << endl;
        currentState = Approach;
    }

    // Approach direction is the direction we move towards the ball and through it
    // Only set once
    // Reset when the ball state changes significantly (This is done in python)

    // If we haven't found an approach direction yet
    if (approachDirection == Point(0,0)) {
        if (ball.vel.mag() < *_ballSpeedApproachDirectionCutoff) {
            // Move directly to the ball
            approachDirection = (ball.pos - startInstant.pos).norm();
        } else {
            // Approach the ball from behind
            // TODO: May need to rethink average here since it'll be based on the first ball vel only
            approachDirection = averageBallVel.norm();
        }
    }

    switch (currentState) {
    // Moves from the current location to the ball
    // Controls the robot up until just before the point of contact
    case Approach: {

        // The target position shouldn't be the ball, it should be where the mouth
        // is touching the ball
        // Move to ball position matching ball speed

        // Setup targets for path planner
        Point targetPos = ball.pos - (*_approachDistTarget + Robot_MouthRadius) * approachDirection;
        MotionInstant target(targetPos, averageBallVel + approachDirection.norm() * *_touchDeltaSpeed);
        vector<Point> startEndPoints{startInstant.pos, target.pos};

        // Decrease accel over the entire path
        // In the correct case, only the end will be decreased
        motionConstraints.maxAcceleration *= *_approachAccelScalePercent;

        // Build a path from now to the end
        unique_ptr<Path> path =
            RRTPlanner::generatePath(startEndPoints, obstacles, motionConstraints, startInstant.vel, target.vel);

        path->setDebugText("approaching");

        // Append to the partial path
        if (path) {
            if (partialPath) {
                path = make_unique<CompositePath>(move(partialPath),
                                                  move(path));
                path->setStartTime(prevPath->startTime());
            }

            return make_unique<AngleFunctionPath>(
                move(path), angleFunctionForCommandType(
                    FacePointCommand(ball.pos)));
        }

        // No partial path found
        // Just use the path we created
        unique_ptr<MotionCommand> rrtCommand =
            make_unique<PathTargetCommand>(target);

        auto request = PlanRequest(systemState, startInstant, move(rrtCommand),
                                   robotConstraints, nullptr, obstacles,
                                   dynamicObstacles, planRequest.shellID);
        path = rrtPlanner.run(request);

        return make_unique<AngleFunctionPath>(
            move(path), angleFunctionForCommandType(
                FacePointCommand(ball.pos)));
    }
    // Move through the ball and stop
    // Controls from the point of impact to stop
    case Control: {
        
        // Only plan the path once and run through it
        // Otherwise it will basically push the ball across the field
        if (controlPathCreated && prevPath) {
            return std::move(prevPath);
        }

        controlPathCreated = true;

        // Scale the max acceleration so we don't stop too quickly
        // Set the max speed to the current speed so it stays constant as
        //  we touch the ball and allows the dribbler to get some time to
        //  spin it up to speed
        // Make sure we don't go over our current max speed
        // Shouldn't happen (tm)
        motionConstraints.maxAcceleration *= *_controlAccelScalePercent;
        motionConstraints.maxSpeed = min(averageBallVel.mag() + *_touchDeltaSpeed,
                                         motionConstraints.maxSpeed);

        // Using the current velocity
        // Calculate stopping distance given the acceleration
        float maxAccel = motionConstraints.maxAcceleration;
        float currentSpeed = averageBallVel.mag() + *_touchDeltaSpeed;

        // Assuming const accel going to zero velocity
        // speed / accel gives time to stop
        // speed / 2 is average speed over entire operation
        float stoppingDist = currentSpeed*currentSpeed / (2 * maxAccel);

        // Move through the ball some distance
        // The initial part will be at a constant speed, then it will decelerate to 0 m/s
        double distFromBall = *_stopDistScale * stoppingDist;
        MotionInstant target;
        target.pos = startInstant.pos + distFromBall * startInstant.vel.norm();
        target.vel = Point(0, 0);


        // Try to use the RRTPlanner to generate the path first
        // It reaches the target better for some reason
        vector<Point> startEndPoints{startInstant.pos, target.pos};
        unique_ptr<Path> path =
            RRTPlanner::generatePath(startEndPoints, obstacles, motionConstraints, startInstant.vel, target.vel);

        // Try to use the plan request if the above fails
        // This sometimes doesn't reach the target commanded though 
        if (!path) {
            std::unique_ptr<MotionCommand> rrtCommand =
                std::make_unique<PathTargetCommand>(target);

            auto request = PlanRequest(systemState, startInstant, std::move(rrtCommand),
                                        robotConstraints, nullptr, obstacles,
                                        dynamicObstacles, planRequest.shellID);
            path = rrtPlanner.run(request);
        }

        path->setDebugText("stopping");

        // Make sure that when the path ends, we don't end up spinning around because we
        // hit go past the ball position at the time of path creation
        Point facePt = startInstant.pos + 10 * (ball.pos - startInstant.pos).norm();

        return make_unique<AngleFunctionPath>(
            std::move(path), angleFunctionForCommandType(
                FacePointCommand(facePt)));
    }
    default: {
        std::cout << "WARNING: Invalid state in collect planner. Restarting" << std::endl;
        currentState = Approach;

        // Stop movement until next frame since it's the safest option programmatically
        MotionInstant target(startInstant);
        target.vel = Point(0, 0);

        std::unique_ptr<MotionCommand> rrtCommand =
            std::make_unique<PathTargetCommand>(target);

        auto request = PlanRequest(systemState, startInstant, std::move(rrtCommand),
                                    robotConstraints, nullptr, obstacles,
                                    dynamicObstacles, planRequest.shellID);
        auto path = rrtPlanner.run(request);
        path->setDebugText("Invalid state in collect");

        return make_unique<AngleFunctionPath>(
            std::move(path),
            angleFunctionForCommandType(FacePointCommand(ball.pos)));
    }
    }
}
}