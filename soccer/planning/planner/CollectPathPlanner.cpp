#include "CollectPathPlanner.hpp"
#include <planning/trajectory/PathSmoothing.hpp>
#include <planning/trajectory/VelocityProfiling.hpp>
#include "PathTargetPlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include <Geometry2d/Pose.hpp>
#include "Configuration.hpp"
#include <vector>

//todo(Ethan) delete using namespace std
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

Trajectory CollectPathPlanner::plan(PlanRequest&& planRequest) {
    SystemState& systemState = planRequest.context->state;
    const Ball& ball = systemState.ball;

    const RJ::Time curTime = RJ::now();

    const CollectCommand& command = std::get<CollectCommand>(planRequest.motionCommand);

    // Start state for specified robot
    RobotInstant startInstant{planRequest.start.pose, planRequest.start.velocity, planRequest.start.timestamp};
    RobotInstant partialStartInstant = startInstant;

    // All the max velocity / acceleration constraints for translation /
    // rotation
    RobotConstraints robotConstraints = planRequest.constraints;
    MotionConstraints& motionConstraints = robotConstraints.mot;

    // Previous RRT path from last iteration
    //todo(Ethan) move here?
    Trajectory prevPath = planRequest.prevTrajectory;

    // The small beginning part of the previous path
    Trajectory partialPath({});

    // The is from the original robot position to the ball
    // We only care about the replan lead time from the current pos in the path
    // to the intercept point
    RJ::Seconds timeIntoPreviousPath;

    // How much of the future we are devoting to the partial path
    // 0ms unless we have a partial path, then it's a partialReplanLeadTime
    RJ::Seconds partialPathTime = 0ms;

    // How much of the previous path to steal
    const RJ::Seconds partialReplanLeadTime = PathTargetPlanner::getPartialReplanLeadTime();

    // Check and see if we should reset the entire thing if we are super far off
    // course or the ball state changes significantly
    checkSolutionValidity(ball, startInstant, prevPath);

    // Change start instant to be the partial path end instead of the robot
    // current location if we actually have already calculated a path the frame
    // before
    if (!prevPath.empty()) {
        timeIntoPreviousPath = curTime - prevPath.begin_time();

        // Make sure we still have time in the path to replan and correct
        // since it's likely that the old path is slightly off
        //
        // ---|----------------|-----------------|
        // TimeNow     EndPartialPrevPath  FinalTargetPoint
        //                     |-----------------|
        //          Amount of the path we can change this iteration
        if (timeIntoPreviousPath <
                prevPath.duration() - 2 * partialReplanLeadTime &&
            timeIntoPreviousPath > 0ms) {
            partialPath = prevPath.subTrajectory(
                0ms, timeIntoPreviousPath + partialReplanLeadTime);
            partialPathTime = partialPath.duration() - timeIntoPreviousPath;
            partialStartInstant = partialPath.last();
        }
    }

    // Initialize the filter to the ball velocity so there's less ramp up
    if (!averageBallVelInitialized) {
        averageBallVel = ball.vel;
        averageBallVelInitialized = true;
    } else {
        averageBallVel = *_targetPointAveragingGain * averageBallVel +
                         (1 - *_targetPointAveragingGain) * ball.vel; //todo(Ethan) use LPF function here?
    }

    // Approach direction is the direction we move towards the ball and through
    // it
    if (ball.vel.mag() < *_ballSpeedApproachDirectionCutoff) {
        // Move directly to the ball
        approachDirection = (ball.pos - startInstant.pose.position()).norm();
    } else {
        // Approach the ball from behind
        approachDirection = averageBallVel.norm();
    }

    // Check if we should transition to control from approach
    processStateTransition(ball, startInstant, prevPath,
                           timeIntoPreviousPath);


    switch (currentState) {
        // Moves from the current location to the slow point of approach
        case CourseApproach: {
            return courseApproach(planRequest, startInstant, std::move(prevPath));
        }
        // Moves from the slow point of approach to just before point of contact
        case FineApproach: {
            return fineApproach(planRequest, startInstant, std::move(prevPath));
        }
        // Move through the ball and stop
        case Control: {
            return control(planRequest, partialStartInstant,
                                     std::move(prevPath),
                                     std::move(partialPath));
        }
        default: {
            return invalid(planRequest);
        }
    }
}

void CollectPathPlanner::checkSolutionValidity(
    const Ball& ball, const RobotInstant& startInstant, const Trajectory& prevPath) {
    bool nearBall = (ball.pos - startInstant.pose.position()).mag() <
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
    const Ball& ball, const RobotInstant& startInstant,
    const Trajectory& prevPath, const RJ::Seconds& timeIntoPreviousPath) {
    // Do the transitions
    float dist = (startInstant.pose.position() - ball.pos).mag() - Robot_MouthRadius;
    float speedDiff =
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

Trajectory CollectPathPlanner::courseApproach(
    const PlanRequest& planRequest, const RobotInstant& startInstant,
    Trajectory&& prevPath) {
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
    //todo(EThan) verify this target
    RobotInstant targetSlow(Pose(pathCourseTarget, pathCourseTarget.angleTo(ball.pos)),
            Twist(targetSlowVel, 0), startInstant.stamp);
    PathTargetCommand rrtCommand{targetSlow};

    auto request = PlanRequest(
        planRequest.context, RobotState{startInstant.pose, startInstant.velocity, startInstant.stamp}, rrtCommand,
        planRequest.constraints, std::move(prevPath), planRequest.obstacles,planRequest.shellID);

    Trajectory coursePath = rrtPlanner.plan(std::move(request));

    // Build a path from now to the slow point
    std::function<double(Point,Point,double)> angleFunction =
        [&](Point pos, Point vel, double angle) {
            return vel.angle();
        };
    PlanAngles(coursePath, RobotState{startInstant.pose, startInstant.velocity, startInstant.stamp}, angleFunction, request.constraints.rot);
    coursePath.setDebugText("Course");
    return std::move(coursePath);
}

Trajectory CollectPathPlanner::fineApproach(
    const PlanRequest& planRequest, const RobotInstant& startInstant,
    Trajectory&& prevPath) {
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

    //todo(Ethan) fix timestamp? fix target angle?
    RobotInstant targetHit(Pose(targetHitPos, targetHitPos.angleTo(ball.pos)), Twist(targetHitVel, 0), planRequest.start.timestamp);

    // Decrease accel at the end so we more smoothly touch the ball
    motionConstraintsHit.maxAcceleration *= *_approachAccelScalePercent;
    // Prevent a last minute accel at the end if the approach dist allows for
    // acceleration in the trapezoid
    motionConstraintsHit.maxSpeed =
        min(targetHitVel.mag(), motionConstraintsHit.maxSpeed);

    DirectPathTargetCommand directCommand{targetHit};

    auto request = PlanRequest(
        planRequest.context, RobotState{startInstant.pose, startInstant.velocity, startInstant.stamp}, directCommand,
        robotConstraintsHit, std::move(prevPath), planRequest.obstacles,planRequest.shellID);

    Trajectory pathHit = directPlanner.plan(std::move(request));

    std::function<double(Point,Point,double)> angleFunction =
            [&](Point pos, Point vel, double angle) {
                return pos.angleTo(ball.pos);
            };
    PlanAngles(pathHit, RobotState{startInstant.pose, startInstant.velocity, startInstant.stamp}, angleFunction, planRequest.constraints.rot);
    pathHit.setDebugText("Fine");
    return std::move(pathHit);
}

Trajectory CollectPathPlanner::control(
    const PlanRequest& planRequest, const RobotInstant& startInstant,
    Trajectory&& prevPath, Trajectory&& partialPath) {
    const Ball& ball = planRequest.context->state.ball;
    const ShapeSet& obstacles = planRequest.obstacles;
    RobotConstraints robotConstraints = planRequest.constraints;
    MotionConstraints& motionConstraints = robotConstraints.mot;
    Point startPoint = planRequest.start.pose.position();

    // Only plan the path once and run through it
    // Otherwise it will basically push the ball across the field
    if (controlPathCreated && !prevPath.empty()) {
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
    if (averageBallVel.angleBetween((ball.pos - startPoint)) > 3.14 / 2) {
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
        startPoint +
        distFromBall * (ball.pos - startPoint +
                        velocityScale * averageBallVel * nonZeroVelTimeDelta)
                           .norm();
    target.vel = Point(0, 0);

    // Try to use the RRTPlanner to generate the path first
    // It reaches the target better for some reason
    vector<Point> startEndPoints{startPoint, target.pos};
    Point vi = startInstant.velocity.linear(), vf = target.vel;
    BezierPath bezier(startEndPoints, vi, vf, motionConstraints);
    Trajectory path = ProfileVelocity(bezier, vi.mag(), vf.mag(), motionConstraints);

    // Make sure that when the path ends, we don't end up spinning around
    // because we hit go past the ball position at the time of path creation
    Point facePt = startPoint +
                   10 * (target.pos - startPoint)
                           .norm();  // startInstant.pos + 10 * (ball.pos -
    // startInstant.pos).norm();
    std::function<double(Point, Point, double)> angleFunction =
            [&](Point pos, Point vel, double angle) -> double {
                return pos.angleTo(facePt);
            };
    PlanAngles(path, RobotState{startInstant.pose, startInstant.velocity, startInstant.stamp}, angleFunction, robotConstraints.rot);



    planRequest.context->debug_drawer.drawLine(
        Segment(startPoint,
                startPoint + (target.pos - startPoint) * 10),
        QColor(255, 255, 255), "Control");

    // Try to use the plan request if the above fails
    // This sometimes doesn't reach the target commanded though
    if (path.empty()) {
        Pose targetPose{target.pos, target.pos.angleTo(facePt)};
        Twist targetVel{target.vel, 0};
        RobotInstant pathTargetGoal{targetPose, targetVel, RJ::now()};
        //todo(Ethan) fix timestamp?
        PathTargetCommand rrtCommand{pathTargetGoal};

        auto request = PlanRequest(
            planRequest.context, RobotState{startInstant.pose, startInstant.velocity, startInstant.stamp}, rrtCommand,
            robotConstraints, Trajectory{{}}, obstacles,
            planRequest.shellID);
        path = rrtPlanner.plan(std::move(request));

        if (!partialPath.empty()) {
            path = Trajectory{std::move(partialPath), std::move(path)};
        }
    }

    path.setDebugText("Control");
    //todo(Ethan) fix these veriable names e.g. path --> trajectory
    return std::move(path);
}

Trajectory CollectPathPlanner::invalid(
    const PlanRequest& planRequest) {
    std::cout << "WARNING: Invalid state in collect planner. Restarting"
              << std::endl;
    currentState = CourseApproach;

    // Stop movement until next frame since it's the safest option
    // programmatically
    RobotInstant targetInstant{planRequest.start.pose, Twist{}, planRequest.start.timestamp};
    PathTargetCommand rrtCommand{targetInstant};
    RobotState startRobotState{planRequest.start.pose, planRequest.start.velocity, planRequest.start.timestamp};
    const Ball& ball = planRequest.context->state.ball;

    auto request = PlanRequest(
        planRequest.context, startRobotState, rrtCommand,
        planRequest.constraints, Trajectory{{}}, planRequest.obstacles,
        planRequest.shellID);
    auto path = rrtPlanner.plan(std::move(request));

    std::function<double(Point,Point,double)> angleFunction =
            [&](Point pos, Point vel, double angle) {
                return pos.angleTo(ball.pos);
            };
    PlanAngles(path, startRobotState, angleFunction, request.constraints.rot);
    path.setDebugText("Invalid state in collect");
    return std::move(path);
}

template <typename T>
T CollectPathPlanner::applyLowPassFilter(const T& oldValue, const T& newValue,
                                         double gain) {
    return gain * newValue + (1 - gain) * oldValue;
}
}  // namespace Planning
