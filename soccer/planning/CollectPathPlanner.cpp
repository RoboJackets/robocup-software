#include "CollectPathPlanner.hpp"

#include "CompositePath.hpp"
#include "MotionInstant.hpp"
#include "Configuration.hpp"
#include "Constants.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

// All the config stuff here

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

    // Obstacle list with ball
    ShapeSet& obstaclesWBall = obstacles;
    obstaclesWBall.add(make_shared<Circle>(ball.predict(curTime).pos, .1));

    // Previous angle path from last iteration
    AngleFunctionPath* prevAnglePath =
        dynamic_cast<AngleFunctionPath*>(planRequest.prevPath.get());

    // Previous RRT path from last iteration
    unique_ptr<Path> prevPath;

    if (prevAnglePath && prevAnglePath->path) {
        prevPath = move(prevAnglePath->path);
    }

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

    // TODO: Decide whether to get an average of ball vel approach 

    // The cutoff ball speed to approach from behind the ball movement or
    // to just directly move in
    const float ballSpeedDirectCutoff = 0.1; // m/s

    // Percent of max accel to use
    const float approachAccelScalePercent = 0.5; // %

    const float distCutoffToControl = 0.07; // m
    const float velCutoffToControl = 1; // m/s

    // Gain on the averaging function to smooth the target point to intercept
    // This is due to the high flucations in the ball velocity frame to frame
    // a*newPoint + (1-a)*oldPoint
    // The lower the number, the less noise affects the system, but the slower it responds to changes
    // The higher the number, the more noise affects the system, but the faster it responds to changes
    const float targetPointAveragingGain = .8;

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

    if (!firstTargetFound) {
        velocityTarget = ball.vel;
        firstTargetFound = true;
    } else {
        velocityTarget = targetPointAveragingGain * velocityTarget + 
                         (1 - targetPointAveragingGain) * ball.vel;
    }

    // Do the transitions
    float dist = (startInstant.pos - ball.pos).mag();
    float speedDiff = (startInstant.vel.mag() - velocityTarget.mag());

    // If we are close enough to the ball and almost the same speed, start slowing down
    // TODO: Check for ball sense
    if (dist < distCutoffToControl && speedDiff < velCutoffToControl && currentState == Approach) {
        currentState = Control;

        std::cout << "Transition to control" << std::endl;
    }

    motionConstraints.maxAcceleration *= approachAccelScalePercent;

    switch (currentState) {
    case Approach: {

        // The target position shouldn't be the ball, it should be where the mouth
        // is touching the ball
        // Move to ball position matching ball speed
        Point approachDirection = -ball.vel;

        // If it's almost 0, approach 
        if (approachDirection.magsq() < 0.01) {
            approachDirection = (startInstant.pos - ball.pos).norm();
        } else {
            approachDirection = approachDirection.norm();
        }

        Point targetPos = ball.pos + approachDirection * 0.09;
        

        MotionInstant target(targetPos, velocityTarget);
        vector<Point> startEndPoints{startInstant.pos, target.pos};

        unique_ptr<Path> path =
            RRTPlanner::generatePath(startEndPoints, obstacles, motionConstraints, startInstant.vel, target.vel);

        path->setDebugText("approaching");

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

        // No point found
        // just move to ball
        unique_ptr<MotionCommand> rrtCommand =
            make_unique<PathTargetCommand>(target);

        auto request = PlanRequest(systemState, startInstant, move(rrtCommand),
                                   robotConstraints, nullptr, obstacles,
                                   dynamicObstacles, planRequest.shellID);
        path = rrtPlanner.run(request);

        return make_unique<AngleFunctionPath>(
            move(path),
            angleFunctionForCommandType(FacePointCommand(ball.pos)));
    }
    case Control: {
        // Stop movement

        // Using the current velocity
        // Calculate stopping point along the ball path
        float maxAccel = motionConstraints.maxAcceleration;
        float currentSpeed = startInstant.vel.mag();

        // Assuming const accel going to zero velocity
        // speed / accel gives time to stop
        // speed / 2 is average time over the entire operation
        float stoppingDist = currentSpeed*currentSpeed / (2 * maxAccel);

        MotionInstant target(startInstant);
        target.vel = Point(0, 0);

        std::unique_ptr<MotionCommand> rrtCommand =
            std::make_unique<PathTargetCommand>(target);

        auto request = PlanRequest(systemState, startInstant, std::move(rrtCommand),
                                    robotConstraints, nullptr, obstacles,
                                    dynamicObstacles, planRequest.shellID);
        auto path = rrtPlanner.run(request);
        path->setDebugText("stopping");

        return make_unique<AngleFunctionPath>(
            std::move(path),
            angleFunctionForCommandType(FacePointCommand(ball.pos)));
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