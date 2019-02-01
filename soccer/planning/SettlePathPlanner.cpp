#include "SettlePathPlanner.hpp"
#include "CompositePath.hpp"
#include "MotionInstant.hpp"
#include "Configuration.hpp"
#include "Constants.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

// REGISTER_CONFIGURABLE(SettlePathPlanner);

// ConfigDouble* SettlePathPlanner::_ballSpeedPercentForDampen;
// ConfigDouble* SettlePathPlanner::_minSpeedToIntercept;
// ConfigDouble* SettlePathPlanner::_maxAngleOffBallForDampen;
// ConfigDouble* SettlePathPlanner::_searchStartTime;
// ConfigDouble* SettlePathPlanner::_searchEndTime;
// ConfigDouble* SettlePathPlanner::_searchIncTime;

// void SettlePathPlanner::createConfiguration(Configuration* cfg) {
//     _ballSpeedPercentForDampen =
//         new ConfigDouble(cfg, "SettlePathPlanner/ballSpeedPercentForDampen", 0.1); // %
//     _minSpeedToIntercept =
//         new ConfigDouble(cfg, "SettlePathPlanner/minSpeedToIntercept", 0.1); // m/s
//     _maxAngleOffBallForDampen =
//         new ConfigDouble(cfg, "SettlePathPlanner/maxAngleOffBallForDampen", 45); // Deg
//     _searchStartTime=
//         new ConfigDouble(cfg, "SettlePathPlanner/searchStartTime", 0.1); // Seconds
//     _searchEndTime =
//         new ConfigDouble(cfg, "SettlePathPlanner/searchEndTime", 6.0); // Seconds
//     _searchIncTime =
//         new ConfigDouble(cfg, "SettlePathPlanner/searchIncTime", 0.2); // Seconds
// }

bool SettlePathPlanner::shouldReplan(const PlanRequest& planRequest) const {
    // Replan whenever the ball changes paths
    // or every X amount of time
    // Also reset the firstTargetPointFound
    return true;
}

std::unique_ptr<Path> SettlePathPlanner::run(PlanRequest& planRequest) {
    SystemState& systemState = planRequest.systemState;
    const Ball& ball = systemState.ball;

    const RJ::Time curTime = RJ::now();

    const SettleCommand& command =
    dynamic_cast<const SettleCommand&>(*planRequest.motionCommand);

    // The direction we will try and bounce the ball when we dampen it to
    // speed up actions after capture
    targetFinalCaptureDirectionPos = command.target;

    // Start state for the specified robot
    MotionInstant& startInstant = planRequest.start;
    // All the max velocity / acceleration constraints for translation / rotation
    const MotionConstraints& motionConstraints = planRequest.constraints.mot;
    const RobotConstraints& robotConstraints = planRequest.constraints;
    // List of obstacles
    Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    std::vector<DynamicObstacle>& dynamicObstacles = planRequest.dynamicObstacles;

    // Obstacle list with circle around ball 
    Geometry2d::ShapeSet& obstaclesWBall = obstacles;
    obstaclesWBall.add(
        make_shared<Circle>(ball.predict(curTime).pos, .1));

    // Previous angle path from last iteration
    AngleFunctionPath* prevAnglePath = 
        dynamic_cast<AngleFunctionPath*>(planRequest.prevPath.get());

    // Previous RRT path from last iteration
    std::unique_ptr<Path> prevPath;

    if (prevAnglePath && prevAnglePath->path) {
        prevPath = std::move(prevAnglePath->path);
    }

    // The small beginning part of the previous path
    std::unique_ptr<Path> partialPath = nullptr;

    // The path is from the original robot position to the intercept point
    // We only care about the replan lead time from the current pos in the path
    // to the intercept point
    RJ::Seconds timeIntoPreviousPath;

    // How much of the future we are devoting to the partial path
    // 0 unless we have a partial path, then it's partialReplanLeadTime
    RJ::Seconds partialPathTime = 0ms;

    // How much of the previous path to steal
    const RJ::Seconds partialReplanLeadTime = RRTPlanner::getPartialReplanLeadTime();

    // How much of the ball speed to use to dampen the bounce
    const float ballSpeedPercentForDampen = 1 / 100.0; // %

    // Ball speed cutoff to decide when to catch it from behind or get in front of it
    const float minSpeedToIntercept = 0.1; // m/s

    // Max angle from the ball vector when trying to bounce the ball
    // on dampen when trying to speed up actions after captures
    const float maxAngleoffBallForDampen = 45; // deg

    // Search settings when trying to find the correct intersection location
    // between the fast moving ball and our collecting robot
    const RJ::Seconds searchStartTime = RJ::Seconds(0.1);
    const RJ::Seconds searchEndTime = RJ::Seconds(10);
    const RJ::Seconds searchIncTime = RJ::Seconds(0.2);

    // Gain on the averaging function to smooth the target point to intercept
    // This is due to the high flucations in the ball velocity frame to frame
    // a*newPoint + (1-a)*oldPoint
    // The lower the number, the less noise affects the system, but the slower it responds to changes
    // The higher the number, the more noise affects the system, but the faster it responds to changes
    const float targetPointAveragingGain = .8;

    // Change start instant to be the partial path end instead of the robot current location
    // if we actually have already calculated a path the frame before
    if (prevPath && currentState != Complete) {
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

    // State transitions
    // Intercept -> Dampen, PrevPath and almost at the end of the path
    // Dampen -> Complete, PrevPath and almost slowed down to 0?
    if (prevPath) {
        timeIntoPreviousPath = curTime - prevPath->startTime();
        Geometry2d::Line ballMovementLine(ball.pos, ball.pos + ball.vel);
        std::unique_ptr<Path> pathSoFar = prevPath->subPath(0ms, timeIntoPreviousPath);
        
        float botDistToBallMovementLine = ballMovementLine.distTo(pathSoFar->end().motion.pos);

        // Intercept -> Dampen
        //  Almost at the end of the path and in the path of the ball
        // TODO: Check ball sense?
        if (timeIntoPreviousPath >= prevPath->getDuration() - 2*partialReplanLeadTime &&
            botDistToBallMovementLine < Robot_Radius &&
            currentState == Intercept) {
            
            startInstant = pathSoFar->end().motion;
            currentState = Dampen;
            std::cout << "Transitioned to dampen" << std::endl;
            std::cout << timeIntoPreviousPath << " : " << prevPath->getDuration() << std::endl;
        }
        // Dampen -> Complete
        //  Almost at the end of the path
        //  TODO: Make sure ball was hit or slowed down
        else if (timeIntoPreviousPath >= prevPath->getDuration() - 2*partialReplanLeadTime &&
            currentState == Dampen && false) {
            
            currentState = Complete;
            std::cout << "Transitioned to complete" << std::endl;
        }
    }

    switch (currentState) {
    case Intercept: {
        // Try find best point to intercept using old method
        // where we check ever X distance along the ball velocity vector
        // TODO: Try the pronav algorithm
        for (float dist = 0; dist < 5; dist += 0.05) {
            Point ballVelIntercept;
            RJ::Seconds t = RJ::Seconds(ball.estimateTimeTo(ball.pos + ball.vel.normalized()*dist, &ballVelIntercept) - curTime);
            MotionInstant targetRobotIntersection(ballVelIntercept);
            std::vector<Geometry2d::Point> startEndPoints{startInstant.pos, targetRobotIntersection.pos};

            // TODO: Take the targetFinalCaptureDirection into account
            // Use the mouth to center vector, rotate by X degrees
            // Take the delta between old and new mouth vector and move
            // targetRobotIntersection by that amount

            // TODO: Improve velocity target
            //       May be able to just dampen here instead of another state
            targetRobotIntersection.vel = Point(0, 0);

            std::unique_ptr<Path> path =
                RRTPlanner::generatePath(startEndPoints, obstacles, motionConstraints, startInstant.vel, targetRobotIntersection.vel);

            if (path) {
                RJ::Seconds timeOfArrival = path->getDuration();

                // We can reach the target point before 
                if (timeOfArrival + partialPathTime <= t) {
                    // No path found yet so there is nothing to average
                    if (!firstTargetPointFound) {
                        interceptTarget = targetRobotIntersection.pos;
                        averagePathTime = t;

                        firstTargetPointFound = true;
                    // Average this calculation with the previous ones
                    } else {
                        interceptTarget = targetPointAveragingGain * targetRobotIntersection.pos +
                                        (1 - targetPointAveragingGain) * interceptTarget;

                        averagePathTime = targetPointAveragingGain * t +
                                        (1 - targetPointAveragingGain) * averagePathTime;
                    }
                                        
                    break;
                }
            }
        }

        // There is some valid interception point to go towards
        if (firstTargetPointFound) {
            // Try and use the previous path for the first part so it will actually make the initial turn
            MotionInstant targetRobotIntersection(interceptTarget);
            std::vector<Geometry2d::Point> startEndPoints{startInstant.pos, targetRobotIntersection.pos};
            targetRobotIntersection.vel = ballSpeedPercentForDampen*ball.vel;

            std::unique_ptr<Path> interceptAndDampenStart =
                    RRTPlanner::generatePath(startEndPoints, obstacles, motionConstraints, startInstant.vel, targetRobotIntersection.vel);

            if (interceptAndDampenStart) {
                RJ::Seconds timeOfArrival = interceptAndDampenStart->getDuration();
                interceptAndDampenStart->setDebugText(QString::number(timeOfArrival.count()) + " : " + QString::number(averagePathTime.count()));

                if (partialPath) {
                    interceptAndDampenStart = std::make_unique<CompositePath>(std::move(partialPath),
                                                        std::move(interceptAndDampenStart));
                    interceptAndDampenStart->setStartTime(prevPath->startTime());
                }

                return make_unique<AngleFunctionPath>(
                    std::move(interceptAndDampenStart), angleFunctionForCommandType(
                        FacePointCommand(ball.pos)));
            }
        }

        // No point found
        // Try to move to the closest point in the ball vel line
        Line ballMovementLine(ball.pos, ball.pos + ball.vel);
        MotionInstant target(ballMovementLine.nearestPoint(startInstant.pos), Point(0,0));

        std::unique_ptr<MotionCommand> rrtCommand =
            std::make_unique<PathTargetCommand>(target);

        auto request = PlanRequest(systemState, startInstant, std::move(rrtCommand),
                                robotConstraints, nullptr, obstacles,
                                dynamicObstacles, planRequest.shellID);
        auto path = rrtPlanner.run(request);
        path->setDebugText("Gives ups");

        return make_unique<AngleFunctionPath>(
            std::move(path),
            angleFunctionForCommandType(FacePointCommand(ball.pos)));
    }
    case Dampen: {
        // Intercept ends with a % ball velocity in the direction of the ball movement
        // Slow down once ball is nearby to 0 m/s

        // Try to slow down as fast as possible to 0 m/s along the ball path
        // We have to do position control since we want to stay in the line of the ball while
        // we do this. If we did velocity, we have very little control of where on the field it
        // is without some other position controller.

        // Uses constant acceleration to create a linear velocity profile
        // TODO: Implement own Path planner to move in a custom velocity profile
        //       Linear acceleration?
        //       Custom acceleration?

        // Using the current velocity
        // Calculate stopping point along the ball path
        float maxAccel = motionConstraints.maxAcceleration;
        float currentSpeed = startInstant.vel.mag();

        // Assuming const accel going to zero velocity
        // speed / accel gives time to stop
        // speed / 2 is average time over the entire operation
        float stoppingDist = currentSpeed*currentSpeed / (2 * maxAccel);

        Geometry2d::Point ballMovementDir(ball.vel.normalized());
        Geometry2d::Line ballMovementLine(ball.pos, ball.pos + ballMovementDir);
        Geometry2d::Point nearestPointToRobot = ballMovementLine.nearestPoint(startInstant.pos);
        float distToBallMovementLine = (startInstant.pos - nearestPointToRobot).mag();

        // Default to just moving to the closest point on the line
        Geometry2d::Point finalStoppingPoint(nearestPointToRobot);

        // Make sure we are actually moving before we start trying to optimize stuff
        if (stoppingDist >= 0.01f) {
            // The closer we are to the line, the less we should move into the line to stop overshoot
            float percentStoppingDistToBallMovementLine = distToBallMovementLine / stoppingDist;

            // 0% should be just stopping at stoppingDist down the ball movement line from the nearestPointToRobot
            // 100% or more should just be trying to get to the nearestPointToRobot (Default case)
            if (percentStoppingDistToBallMovementLine < 1) {
                // c^2 - a^2 = b^2
                // c is stopping dist, a is dist to ball line
                // b is dist down ball line
                float distDownBallMovementLine = sqrt(stoppingDist*stoppingDist - distToBallMovementLine*distToBallMovementLine);
                finalStoppingPoint = nearestPointToRobot + distDownBallMovementLine * ballMovementDir;
            }
        }
        
        // Target stopping point with 0 speed.
        MotionInstant finalStoppingMotion(finalStoppingPoint, Geometry2d::Point(0,0));
        std:unique_ptr<MotionCommand> rrtCommand = 
            std::make_unique<DirectPathTargetCommand>(finalStoppingMotion);

        auto request = PlanRequest(systemState, startInstant, std::move(rrtCommand),
                                   robotConstraints, nullptr, obstacles,
                                dynamicObstacles, planRequest.shellID);
        std::unique_ptr<Path> dampenEnd = directPlanner.run(request);
        dampenEnd->setDebugText("Damping");

        if (partialPath) {
            dampenEnd = std::make_unique<CompositePath>(std::move(partialPath),
                                                std::move(dampenEnd));
            dampenEnd->setStartTime(prevPath->startTime());
        }

        return make_unique<AngleFunctionPath>(
            std::move(dampenEnd),
            angleFunctionForCommandType(FacePointCommand(ball.pos)));
    }
    case Complete: {
        // Set movement target at current location
        MotionInstant target(startInstant);
        target.vel = Point(0, 0);

        std::unique_ptr<MotionCommand> rrtCommand =
            std::make_unique<PathTargetCommand>(target);

        auto request = PlanRequest(systemState, startInstant, std::move(rrtCommand),
                                    robotConstraints, nullptr, obstacles,
                                    dynamicObstacles, planRequest.shellID);
        auto path = rrtPlanner.run(request);
        path->setDebugText("Completed Settle");

        return make_unique<AngleFunctionPath>(
            std::move(path),
            angleFunctionForCommandType(FacePointCommand(ball.pos)));
    }
    default: {
        std::cout << "Error: Invalid state in settle planner. Restarting" << std::endl;
        currentState = Intercept;

        // Stop movement until next frame since it's the safest option programmatically
        MotionInstant target(startInstant);
        target.vel = Point(0, 0);

        std::unique_ptr<MotionCommand> rrtCommand =
            std::make_unique<PathTargetCommand>(target);

        auto request = PlanRequest(systemState, startInstant, std::move(rrtCommand),
                                    robotConstraints, nullptr, obstacles,
                                    dynamicObstacles, planRequest.shellID);
        auto path = rrtPlanner.run(request);
        path->setDebugText("Invalid state in settle");

        return make_unique<AngleFunctionPath>(
            std::move(path),
            angleFunctionForCommandType(FacePointCommand(ball.pos)));
    }
    }
}
}

