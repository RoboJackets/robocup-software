#include "CollectPlanner.hpp"

#include "Configuration.hpp"
#include "Constants.hpp"
#include "planning/trajectory/RRTUtil.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

REGISTER_CONFIGURABLE(CollectPlanner);

ConfigDouble* CollectPlanner::_ballSpeedApproachDirectionCutoff;
ConfigDouble* CollectPlanner::_approachAccelScalePercent;
ConfigDouble* CollectPlanner::_controlAccelScalePercent;
ConfigDouble* CollectPlanner::_approachDistTarget;
ConfigDouble* CollectPlanner::_touchDeltaSpeed;
ConfigDouble* CollectPlanner::_velocityControlScale;
ConfigDouble* CollectPlanner::_distCutoffToControl;
ConfigDouble* CollectPlanner::_velCutoffToControl;
ConfigDouble* CollectPlanner::_distCutoffToApproach;
ConfigDouble* CollectPlanner::_stopDistScale;
ConfigDouble* CollectPlanner::_targetPointAveragingGain;

void CollectPlanner::createConfiguration(Configuration* cfg) {
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

Trajectory CollectPlanner::plan(PlanRequest&& planRequest) {
    BallState ball = planRequest.world_state->ball;

    const RJ::Time curTime = RJ::now();

    const CollectCommand& command =
        std::get<CollectCommand>(planRequest.motionCommand);

    // Start state for specified robot
    RobotInstant startInstant = planRequest.start;
    RobotInstant partialStartInstant = startInstant;

    // All the max velocity / acceleration constraints for translation /
    // rotation
    RobotConstraints robotConstraints = planRequest.constraints;
    MotionConstraints& motionConstraints = robotConstraints.mot;

    // List of obstacles
    ShapeSet obstacles;
    std::vector<DynamicObstacle> dynamicObstacles;
    FillObstacles(planRequest, &obstacles, &dynamicObstacles, false);

    // The small beginning part of the previous path
    Trajectory partialPath;

    // The is from the original robot position to the ball
    // We only care about the replan lead time from the current pos in the path
    // to the intercept point
    RJ::Seconds timeIntoPreviousPath;

    // How much of the future we are devoting to the partial path
    // 0ms unless we have a partial path, then it's a partialReplanLeadTime
    RJ::Seconds partialPathTime = 0ms;

    // How much of the previous path to steal
    const RJ::Seconds partialReplanLeadTime(
        Replanner::partialReplanLeadTime());

    // Check and see if we should reset the entire thing if we are super far off
    // course or the ball state changes significantly
    checkSolutionValidity(ball, startInstant);

    // Change start instant to be the partial path end instead of the robot
    // current location if we actually have already calculated a path the frame
    // before
    if (!previous.empty()) {
        timeIntoPreviousPath = curTime - previous.begin_time();

        // Make sure we still have time in the path to replan and correct
        // since it's likely that the old path is slightly off
        //
        // ---|----------------|-----------------|
        // TimeNow     EndPartialPrevPath  FinalTargetPoint
        //                     |-----------------|
        //          Amount of the path we can change this iteration
        if (timeIntoPreviousPath <
            previous.duration() - 2 * partialReplanLeadTime &&
            timeIntoPreviousPath > 0ms) {
            partialPath = previous.subTrajectory(
                0ms, timeIntoPreviousPath + partialReplanLeadTime);
            partialPathTime = partialPath.duration() - timeIntoPreviousPath;
            partialStartInstant = partialPath.last();
        }
    }

    // Initialize the filter to the ball velocity so there's less ramp up
    if (!averageBallVelInitialized) {
        averageBallVel = ball.velocity;
        averageBallVelInitialized = true;
    } else {
        averageBallVel = *_targetPointAveragingGain * averageBallVel +
                         (1 - *_targetPointAveragingGain) * ball.velocity;
    }

    // Approach direction is the direction we move towards the ball and through
    // it
    if (ball.velocity.mag() < *_ballSpeedApproachDirectionCutoff) {
        // Move directly to the ball
        approachDirection = (ball.position - startInstant.pose.position()).norm();
    } else {
        // Approach the ball from behind
        approachDirection = averageBallVel.norm();
    }

    // Check if we should transition to control from approach
    processStateTransition(ball, startInstant);

    switch (currentState) {
        // Moves from the current location to the slow point of approach
        case CourseApproach:
            previous = courseApproach(planRequest, startInstant, obstacles,
                                      dynamicObstacles);
            break;
        // Moves from the slow point of approach to just before point of contact
        case FineApproach:
            previous = fineApproach(planRequest, startInstant, obstacles,
                                  dynamicObstacles);
            break;
        // Move through the ball and stop
        case Control:
            previous = control(planRequest, partialStartInstant,
                             partialPath, obstacles,
                             dynamicObstacles);
            break;
        default:
            previous = invalid(planRequest, obstacles, dynamicObstacles);
            break;
    }

    return previous;
}

void CollectPlanner::checkSolutionValidity(BallState ball,
                                           RobotInstant start) {
    bool nearBall = (ball.position - start.pose.position()).mag() <
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

void CollectPlanner::processStateTransition(BallState ball, RobotInstant startInstant) {
    // Do the transitions
    double dist = (startInstant.pose.position() - ball.position).mag() - Robot_MouthRadius;
    double speedDiff =
        (startInstant.velocity.linear() - averageBallVel).mag() - *_touchDeltaSpeed;

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

Trajectory CollectPlanner::courseApproach(
    const PlanRequest& planRequest, RobotInstant start,
    const Geometry2d::ShapeSet& staticObstacles,
    const std::vector<DynamicObstacle>& dynamicObstacles) {
    BallState ball = planRequest.world_state->ball;

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
        ball.position -
        (*_approachDistTarget + Robot_MouthRadius) * approachDirection;
    Point targetSlowVel =
        averageBallVel + approachDirection * *_touchDeltaSpeed;

    // Force the path to use the same target if it doesn't move too much
    if ((pathCourseTarget - targetSlowPos).mag() >
        (*_approachDistTarget - *_distCutoffToControl) / 2) {
        pathCourseTarget = targetSlowPos;
    }

    RobotInstant targetSlow;
    targetSlow.pose.position() = pathCourseTarget;
    targetSlow.velocity.linear() = targetSlowVel;

    Replanner::PlanParams params {
        start,
        targetSlow,
        staticObstacles,
        dynamicObstacles,
        planRequest.constraints,
        AngleFns::facePoint(ball.position)
    };
    Trajectory coarsePath = rrtPlanner.CreatePlan(params, previous);

    // Build a path from now to the slow point
    coarsePath.setDebugText("course");

    return coarsePath;
}

Trajectory CollectPlanner::fineApproach(
    const PlanRequest& planRequest,
    RobotInstant startInstant,
    const Geometry2d::ShapeSet& staticObstacles,
    const std::vector<DynamicObstacle>& dynamicObstacles) {
    BallState ball = planRequest.world_state->ball;
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
    Point targetHitPos = ball.position - Robot_MouthRadius * approachDirection;
    Point targetHitVel = averageBallVel + approachDirection * *_touchDeltaSpeed;

    RobotInstant targetHit;
    targetHit.pose.position() = targetHitPos;
    targetHit.velocity.linear() = targetHitVel;

    // Decrease accel at the end so we more smoothly touch the ball
    motionConstraintsHit.maxAcceleration *= *_approachAccelScalePercent;
    // Prevent a last minute accel at the end if the approach dist allows for
    // acceleration in the trapezoid
    motionConstraintsHit.maxSpeed =
        std::min(targetHitVel.mag(), motionConstraintsHit.maxSpeed);

    Trajectory pathHit = CreatePath::simple(startInstant,
                                            targetHit,
                                            planRequest.constraints.mot);
    pathHit.setDebugText("fine");
    PlanAngles(pathHit, startInstant, AngleFns::facePoint(ball.position),
               planRequest.constraints.rot);

    return pathHit;
}

Trajectory CollectPlanner::control(
    const PlanRequest& planRequest, RobotInstant start,
    const Trajectory& partialPath,
    const Geometry2d::ShapeSet& staticObstacles,
    const std::vector<DynamicObstacle>& dynamicObstacles) {
    BallState ball = planRequest.world_state->ball;
    RobotConstraints robotConstraints = planRequest.constraints;
    MotionConstraints& motionConstraints = robotConstraints.mot;

    // Only plan the path once and run through it
    // Otherwise it will basically push the ball across the field
    if (controlPathCreated && !previous.empty()) {
        return std::move(previous);
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
    if (averageBallVel.angleBetween((ball.position - start.pose.position())) > 3.14 / 2) {
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
    RobotInstant target;
    target.pose.position() =
        start.pose.position() +
        distFromBall * (ball.position - start.pose.position() +
                        velocityScale * averageBallVel * nonZeroVelTimeDelta)
            .norm();

    // Try to use the RRTPlanner to generate the path first
    // It reaches the target better for some reason
    vector<Point> startEndPoints{start.pose.position(), target.pose.position()};
    Trajectory path = CreatePath::rrt(start, target, motionConstraints, staticObstacles, dynamicObstacles);

    if (planRequest.debug_drawer != nullptr) {
        planRequest.debug_drawer->drawLine(
            Segment(start.pose.position(),
                    start.pose.position() + (target.pose.position() - start.pose.position()) * 10),
            QColor(255, 255, 255), "Control");
    }

#if 0
    // Try to use the plan request if the above fails
    // This sometimes doesn't reach the target commanded though
    if (path.empty()) {
        std::unique_ptr<MotionCommand> rrtCommand =
            std::make_unique<PathTargetCommand>(target);

        auto request = PlanRequest(
            planRequest.context, startInstant, std::move(rrtCommand),
            robotConstraints, nullptr, obstacles, planRequest.dynamicObstacles,
            planRequest.shellID);
        path = rrtPlanner.run(request);

        if (!partialPath.empty()) {
            path = make_unique<CompositePath>(move(partialPath), move(path));
            path->setStartTime(prevPath->startTime());
        }
    }
#endif

    path.setDebugText("stopping");

    // Make sure that when the path ends, we don't end up spinning around
    // because we hit go past the ball position at the time of path creation
    Point facePt = start.pose.position() +
                   10 * (target.pose.position() - start.pose.position())
                       .norm();

    PlanAngles(path, start, AngleFns::facePoint(facePt), robotConstraints.rot);
    return path;
}

Trajectory CollectPlanner::invalid(
    const PlanRequest& planRequest,
    const Geometry2d::ShapeSet& staticObstacles,
    const std::vector<DynamicObstacle>& dynamicObstacles) {
    std::cout << "WARNING: Invalid state in collect planner. Restarting"
              << std::endl;
    currentState = CourseApproach;

    // Stop movement until next frame since it's the safest option
    // programmatically
    RobotInstant target = planRequest.start;
    target.velocity = Geometry2d::Twist::Zero();

    Replanner::PlanParams params {
        planRequest.start,
        target,
        staticObstacles,
        dynamicObstacles,
        planRequest.constraints,
        AngleFns::facePoint(planRequest.world_state->ball.position)
    };
    Trajectory path = rrtPlanner.CreatePlan(params, previous);
    path.setDebugText("Invalid state in collect");

    return path;
}

template <typename T>
T CollectPlanner::applyLowPassFilter(const T& oldValue, const T& newValue,
                                         double gain) {
    return gain * newValue + (1 - gain) * oldValue;
}
}  // namespace Planning
