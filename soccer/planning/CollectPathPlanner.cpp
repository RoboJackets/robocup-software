#include "CollectPathPlanner.hpp"

#include "CompositePath.hpp"
#include "Configuration.hpp"
#include "Constants.hpp"
#include "MotionInstant.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

REGISTER_CONFIGURABLE(CollectPathPlanner);

ConfigDouble* CollectPathPlanner::_ballSpeedApproachDirectionCutoff;
ConfigDouble* CollectPathPlanner::_approachAccelScalePercent;
ConfigDouble* CollectPathPlanner::_controlAccelScalePercent;
ConfigDouble* CollectPathPlanner::_approachDistTarget;
ConfigDouble* CollectPathPlanner::_touchDeltaSpeed;
ConfigDouble* CollectPathPlanner::_velocityControlScale;
ConfigDouble* CollectPathPlanner::_distCutoffToControl;
ConfigDouble* CollectPathPlanner::_velCutoffToControl;
ConfigDouble* CollectPathPlanner::_distCutoffToApproach;
ConfigDouble* CollectPathPlanner::_stopDistScale;
ConfigDouble* CollectPathPlanner::_targetPointAveragingGain;

void CollectPathPlanner::createConfiguration(Configuration* cfg) {
    _ballSpeedApproachDirectionCutoff = new ConfigDouble(
        cfg, "Capture/Collect/ballSpeedApproachDirectionCutoff", 0.1);
    _approachAccelScalePercent =
        new ConfigDouble(cfg, "Capture/Collect/approachAccelScalePercent", 0.7);
    _controlAccelScalePercent =
        new ConfigDouble(cfg, "Capture/Collect/controlAccelScalePercent", 0.8);
    _approachDistTarget =
        new ConfigDouble(cfg, "Capture/Collect/approachDistTarget", 0.04);
    _touchDeltaSpeed =
        new ConfigDouble(cfg, "Capture/Collect/touchDeltaSpeed", 0.1);
    _velocityControlScale =
        new ConfigDouble(cfg, "Capture/Collect/velocityControlScale", 1);
    _distCutoffToControl =
        new ConfigDouble(cfg, "Capture/Collect/distCutoffToControl", 0.05);
    _velCutoffToControl =
        new ConfigDouble(cfg, "Capture/Collect/velCutoffToControl", 1);
    _distCutoffToApproach =
        new ConfigDouble(cfg, "Capture/Collect/distCutoffToApproach", 0.3);
    _stopDistScale = new ConfigDouble(cfg, "Capture/Collect/stopDistScale", 1);
    _targetPointAveragingGain =
        new ConfigDouble(cfg, "Capture/Collect/targetPointAveragingGain", 0.8);
}

bool CollectPathPlanner::shouldReplan(const PlanRequest& planRequest) const {
    // Always replan since the ball is very noisy
    return true;
}

std::unique_ptr<Path> CollectPathPlanner::run(PlanRequest& planRequest) {
    SystemState& systemState = planRequest.context->state;
    const Ball& ball = systemState.ball;

    const RJ::Time curTime = RJ::now();

    const CollectCommand& command =
        dynamic_cast<const CollectCommand&>(*planRequest.motionCommand);

    // Start state for specified robot
    MotionInstant& startInstant = planRequest.start;
    MotionInstant partialStartInstant = startInstant;

    // All the max velocity / acceleration constraints for translation /
    // rotation
    RobotConstraints robotConstraints = planRequest.constraints;
    MotionConstraints& motionConstraints = robotConstraints.mot;

    // List of obstacles
    ShapeSet& obstacles = planRequest.obstacles;
    vector<DynamicObstacle>& dynamicObstacles = planRequest.dynamicObstacles;

    // Previous RRT path from last iteration
    unique_ptr<Path>& prevPath = planRequest.prevPath;

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
    const RJ::Seconds partialReplanLeadTime =
        RRTPlanner::getPartialReplanLeadTime();

    // Check and see if we should reset the entire thing if we are super far off
    // course or the ball state changes significantly
    checkSolutionValidity(ball, startInstant, prevPath.get());

    // Change start instant to be the partial path end instead of the robot
    // current location if we actually have already calculated a path the frame
    // before
    if (prevPath) {
        timeIntoPreviousPath = curTime - prevPath->startTime();

        // Make sure we still have time in the path to replan and correct
        // since it's likely that the old path is slightly off
        //
        // ---|----------------|-----------------|
        // TimeNow     EndPartialPrevPath  FinalTargetPoint
        //                     |-----------------|
        //          Amount of the path we can change this iteration
        if (timeIntoPreviousPath <
                prevPath->getDuration() - 2 * partialReplanLeadTime &&
            timeIntoPreviousPath > 0ms) {
            partialPath = prevPath->subPath(
                0ms, timeIntoPreviousPath + partialReplanLeadTime);
            partialPathTime = partialPath->getDuration() - timeIntoPreviousPath;
            partialStartInstant = partialPath->end().motion;
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

    // Approach direction is the direction we move towards the ball and through
    // it
    if (ball.vel.mag() < *_ballSpeedApproachDirectionCutoff) {
        // Move directly to the ball
        approachDirection = (ball.pos - startInstant.pos).norm();
    } else {
        // Approach the ball from behind
        approachDirection = averageBallVel.norm();
    }

    // Check if we should transition to control from approach
    processStateTransition(ball, startInstant, prevPath.get(),
                           timeIntoPreviousPath);

    switch (currentState) {
        // Moves from the current location to the slow point of approach
        case CourseApproach: {
            return std::move(
                courseApproach(planRequest, startInstant, std::move(prevPath)));
        }
        // Moves from the slow point of approach to just before point of contact
        case FineApproach: {
            return std::move(
                fineApproach(planRequest, startInstant, std::move(prevPath)));
        }
        // Move through the ball and stop
        case Control: {
            return std::move(control(planRequest, partialStartInstant,
                                     std::move(prevPath),
                                     std::move(partialPath), obstacles));
        }
        default: { return std::move(invalid(planRequest)); }
    }
}

void CollectPathPlanner::checkSolutionValidity(
    const Ball& ball, const MotionInstant& startInstant, const Path* prevPath) {
    bool nearBall = (ball.pos - startInstant.pos).mag() <
                    *_distCutoffToApproach + *_distCutoffToControl;

    // Check if we need to go back into approach
    //
    // See if we are not near the ball and both almost stopped
    if (!nearBall && currentState == Control) {
        currentState = CourseApproach;
        approachDirectionCreated = false;
        controlPathCreated = false;
    }
}

void CollectPathPlanner::processStateTransition(
    const Ball& ball, const MotionInstant& startInstant,
    const Path* const prevPath, const RJ::Seconds& timeIntoPreviousPath) {
    // Do the transitions
    float dist = (startInstant.pos - ball.pos).mag() - Robot_MouthRadius;
    float speedDiff =
        (startInstant.vel - averageBallVel).mag() - *_touchDeltaSpeed;

    // If we are in range to the slow dist
    if (dist < *_approachDistTarget + Robot_MouthRadius &&
        currentState == CourseApproach) {
        currentState = FineApproach;
    }

    // If we are close enough to the target point near the ball
    // and almost the same speed we want, start slowing down
    // TODO: Check for ball sense?
    if (dist < *_distCutoffToControl && speedDiff < *_velCutoffToControl &&
        currentState == FineApproach) {
        currentState = Control;
    }
}

std::unique_ptr<Path> CollectPathPlanner::courseApproach(
    const PlanRequest& planRequest, const MotionInstant& startInstant,
    std::unique_ptr<Path> prevPath) {
    const Ball& ball = planRequest.context->state.ball;

    // There are two paths that get combined together
    //
    //
    //     |------------------------|-------| (ball)
    // robot pos                 slow pt  hit pt
    //
    // Robot pos is where we are at now
    // Slow point is where we want to start the const velocity approach
    //     This is due to our acceleration being not exact causing us to
    //     perpetually bump the ball away
    // Hit point is where the robot will touch the ball for the first time

    // The target position shouldn't be the ball, it should be where the mouth
    // is touching the ball

    // Setup targets for path planner
    Point targetSlowPos =
        ball.pos -
        (*_approachDistTarget + Robot_MouthRadius) * approachDirection;
    Point targetSlowVel =
        averageBallVel + approachDirection * *_touchDeltaSpeed;

    // Force the path to use the same target if it doesn't move too much
    if ((pathCourseTarget - targetSlowPos).mag() >
        (*_approachDistTarget - *_distCutoffToControl) / 2) {
        pathCourseTarget = targetSlowPos;
    }

    MotionInstant targetSlow(pathCourseTarget, targetSlowVel);
    unique_ptr<MotionCommand> rrtCommand =
        make_unique<PathTargetCommand>(targetSlow);

    auto request = PlanRequest(
        planRequest.context, startInstant, std::move(rrtCommand),
        planRequest.constraints, std::move(prevPath), planRequest.obstacles,
        planRequest.dynamicObstacles, planRequest.shellID);

    unique_ptr<Path> coursePath = rrtPlanner.run(request);

    // Build a path from now to the slow point
    coursePath->setDebugText("course");

    return make_unique<AngleFunctionPath>(
        move(coursePath),
        angleFunctionForCommandType(FacePointCommand(ball.pos)));
}

std::unique_ptr<Path> CollectPathPlanner::fineApproach(
    const PlanRequest& planRequest, const MotionInstant& startInstant,
    std::unique_ptr<Path> prevPath) {
    const Ball& ball = planRequest.context->state.ball;
    RobotConstraints robotConstraintsHit = planRequest.constraints;
    MotionConstraints& motionConstraintsHit = robotConstraintsHit.mot;

    // There are two paths that get combined together
    //
    //
    //     |------------------------|-------| (ball)
    // robot pos                 slow pt  hit pt
    //
    // Robot pos is where we are at now
    // Slow point is where we want to start the const velocity approach
    //     This is due to our acceleration being not exact causing us to
    //     perpetually bump the ball away
    // Hit point is where the robot will touch the ball for the first time

    // The target position shouldn't be the ball, it should be where the mouth
    // is touching the ball

    // Setup targets for path planner
    Point targetHitPos = ball.pos - Robot_MouthRadius * approachDirection;
    Point targetHitVel = averageBallVel + approachDirection * *_touchDeltaSpeed;

    MotionInstant targetHit(targetHitPos, targetHitVel);

    // Decrease accel at the end so we more smoothly touch the ball
    motionConstraintsHit.maxAcceleration *= *_approachAccelScalePercent;
    // Prevent a last minute accel at the end if the approach dist allows for
    // acceleration in the trapezoid
    motionConstraintsHit.maxSpeed =
        min(targetHitVel.mag(), motionConstraintsHit.maxSpeed);

    unique_ptr<MotionCommand> directCommand =
        make_unique<DirectPathTargetCommand>(targetHit);

    auto request = PlanRequest(
        planRequest.context, startInstant, std::move(directCommand),
        robotConstraintsHit, std::move(prevPath), planRequest.obstacles,
        planRequest.dynamicObstacles, planRequest.shellID);

    unique_ptr<Path> pathHit = directPlanner.run(request);
    pathHit->setDebugText("fine");

    return make_unique<AngleFunctionPath>(
        move(pathHit), angleFunctionForCommandType(FacePointCommand(ball.pos)));
}

std::unique_ptr<Path> CollectPathPlanner::control(
    const PlanRequest& planRequest, const MotionInstant& startInstant,
    std::unique_ptr<Path> prevPath, std::unique_ptr<Path> partialPath,
    const ShapeSet& obstacles) {
    const Ball& ball = planRequest.context->state.ball;
    RobotConstraints robotConstraints = planRequest.constraints;
    MotionConstraints& motionConstraints = robotConstraints.mot;

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
    //
    // If the ball is moving towards us (like receiving a pass) just move
    // forward at touchDeltaSpeed
    float currentSpeed = averageBallVel.mag() + *_touchDeltaSpeed;

    float velocityScale = *_velocityControlScale;

    // Moving at us
    if (averageBallVel.angleBetween((ball.pos - startInstant.pos)) > 3.14 / 2) {
        currentSpeed = *_touchDeltaSpeed;
        velocityScale = 0;
    }

    motionConstraints.maxAcceleration *= *_controlAccelScalePercent;
    motionConstraints.maxSpeed =
        min((double)currentSpeed, motionConstraints.maxSpeed);

    // Using the current velocity
    // Calculate stopping distance given the acceleration
    float maxAccel = motionConstraints.maxAcceleration;

    float nonZeroVelTimeDelta = *_approachDistTarget / *_touchDeltaSpeed;

    // Assuming const accel going to zero velocity
    // speed / accel gives time to stop
    // speed / 2 is average speed over entire operation
    float stoppingDist =
        *_approachDistTarget + currentSpeed * currentSpeed / (2 * maxAccel);

    // Move through the ball some distance
    // The initial part will be at a constant speed, then it will decelerate to
    // 0 m/s
    double distFromBall = *_stopDistScale * stoppingDist;
    MotionInstant target;
    target.pos =
        startInstant.pos +
        distFromBall * (ball.pos - startInstant.pos +
                        velocityScale * averageBallVel * nonZeroVelTimeDelta)
                           .norm();
    target.vel = Point(0, 0);

    // Try to use the RRTPlanner to generate the path first
    // It reaches the target better for some reason
    vector<Point> startEndPoints{startInstant.pos, target.pos};
    unique_ptr<Path> path =
        RRTPlanner::generatePath(startEndPoints, obstacles, motionConstraints,
                                 startInstant.vel, target.vel);

    planRequest.context->debug_drawer.drawLine(
        Segment(startInstant.pos,
                startInstant.pos + (target.pos - startInstant.pos) * 10),
        QColor(255, 255, 255), "Control");

    // Try to use the plan request if the above fails
    // This sometimes doesn't reach the target commanded though
    if (!path) {
        std::unique_ptr<MotionCommand> rrtCommand =
            std::make_unique<PathTargetCommand>(target);

        auto request = PlanRequest(
            planRequest.context, startInstant, std::move(rrtCommand),
            robotConstraints, nullptr, obstacles, planRequest.dynamicObstacles,
            planRequest.shellID);
        path = rrtPlanner.run(request);

        if (partialPath) {
            path = make_unique<CompositePath>(move(partialPath), move(path));
            path->setStartTime(prevPath->startTime());
        }
    }

    path->setDebugText("stopping");

    // Make sure that when the path ends, we don't end up spinning around
    // because we hit go past the ball position at the time of path creation
    Point facePt = startInstant.pos +
                   10 * (target.pos - startInstant.pos)
                            .norm();  // startInstant.pos + 10 * (ball.pos -
                                      // startInstant.pos).norm();

    return make_unique<AngleFunctionPath>(
        std::move(path), angleFunctionForCommandType(FacePointCommand(facePt)));
}

std::unique_ptr<Path> CollectPathPlanner::invalid(
    const PlanRequest& planRequest) {
    std::cout << "WARNING: Invalid state in collect planner. Restarting"
              << std::endl;
    currentState = CourseApproach;

    // Stop movement until next frame since it's the safest option
    // programmatically
    MotionInstant target(planRequest.start);
    target.vel = Point(0, 0);

    std::unique_ptr<MotionCommand> rrtCommand =
        std::make_unique<PathTargetCommand>(target);

    auto request = PlanRequest(
        planRequest.context, planRequest.start, std::move(rrtCommand),
        planRequest.constraints, nullptr, planRequest.obstacles,
        planRequest.dynamicObstacles, planRequest.shellID);
    auto path = rrtPlanner.run(request);
    path->setDebugText("Invalid state in collect");

    return make_unique<AngleFunctionPath>(
        std::move(path), angleFunctionForCommandType(FacePointCommand(
                             planRequest.context->state.ball.pos)));
}

template <typename T>
T CollectPathPlanner::applyLowPassFilter(const T& oldValue, const T& newValue,
                                         double gain) {
    return gain * newValue + (1 - gain) * oldValue;
}
}  // namespace Planning
