#include "SettlePathPlanner.hpp"

#include <cmath>

#include "CompositePath.hpp"
#include "MotionInstant.hpp"
#include "Configuration.hpp"
#include "Constants.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

REGISTER_CONFIGURABLE(SettlePathPlanner);

ConfigDouble* SettlePathPlanner::_ballSpeedPercentForDampen;
ConfigDouble* SettlePathPlanner::_searchStartTime;
ConfigDouble* SettlePathPlanner::_searchEndTime;
ConfigDouble* SettlePathPlanner::_searchIncTime;
ConfigDouble* SettlePathPlanner::_interceptBufferTime;
ConfigDouble* SettlePathPlanner::_targetPointGain;
ConfigDouble* SettlePathPlanner::_ballVelGain;
ConfigDouble* SettlePathPlanner::_maxAnglePathTargetChange;
ConfigDouble* SettlePathPlanner::_maxBallVelForPathReset;
ConfigDouble* SettlePathPlanner::_maxBallAngleForReset; 

void SettlePathPlanner::createConfiguration(Configuration* cfg) {
    _ballSpeedPercentForDampen =
        new ConfigDouble(cfg, "Capture/Settle/ballSpeedPercentForDampen", 0.1); // %
    _searchStartTime =
        new ConfigDouble(cfg, "Capture/Settle/searchStartTime", 0.1); // Seconds
    _searchEndTime =
        new ConfigDouble(cfg, "Capture/Settle/searchEndtime", 10.0); // Seconds
    _searchIncTime =
        new ConfigDouble(cfg, "Capture/Settle/searchIncTime", 0.2); // Seconds
    _interceptBufferTime =
        new ConfigDouble(cfg, "Capture/Settle/interceptBufferTime", 0.0); // Seconds
    _targetPointGain =
        new ConfigDouble(cfg, "Capture/Settle/targetPointGain", 0.1);
    _ballVelGain =
        new ConfigDouble(cfg, "Capture/Settle/ballVelGain", 0.1);
    _maxAnglePathTargetChange =
        new ConfigDouble(cfg, "Capture/Settle/maxAnglePathTargetChange", 20); // Deg
    _maxBallVelForPathReset =
        new ConfigDouble(cfg, "Capture/Settle/maxBallVelForPathReset", 2); // m/s
    _maxBallAngleForReset =
        new ConfigDouble(cfg, "Capture/Settle/maxBallAngleForReset", 20); // Deg
}

bool SettlePathPlanner::shouldReplan(const PlanRequest& planRequest) const {
    return true;
}

std::unique_ptr<Path> SettlePathPlanner::run(PlanRequest& planRequest) {
    SystemState& systemState = planRequest.systemState;
    const Ball& ball = systemState.ball;

    const RJ::Time curTime = RJ::now();

    const SettleCommand& command = dynamic_cast<const SettleCommand&>(*planRequest.motionCommand);

    // The direction we will try and bounce the ball when we dampen it to
    // speed up actions after capture
    targetFinalCaptureDirectionPos = command.target;

    // Start state for the specified robot
    MotionInstant& startInstant = planRequest.start;
    // All the max velocity / acceleration constraints for translation / rotation
    const MotionConstraints& motionConstraints = planRequest.constraints.mot;
    const RobotConstraints& robotConstraints = planRequest.constraints;

    // List of obstacles
    ShapeSet& obstacles = planRequest.obstacles;
    vector<DynamicObstacle>& dynamicObstacles = planRequest.dynamicObstacles;

    // Obstacle list with circle around ball 
    ShapeSet& obstaclesWBall = obstacles;
    obstaclesWBall.add(make_shared<Circle>(ball.predict(curTime).pos, .1));

    // Previous RRT path from last iteration
    std::unique_ptr<Path>& prevPath = planRequest.prevPath;

    // The small beginning part of the previous path
    unique_ptr<Path> partialPath = nullptr;

    // The path is from the original robot position to the intercept point
    // We only care about the replan lead time from the current pos in the path
    // to the intercept point
    RJ::Seconds timeIntoPreviousPath;

    // How much of the future we are devoting to the partial path
    // 0ms unless we have a partial path, then it's partialReplanLeadTime
    RJ::Seconds partialPathTime = 0ms;

    // How much of the previous path to steal
    const RJ::Seconds partialReplanLeadTime = RRTPlanner::getPartialReplanLeadTime();

    // Max angle from the ball vector when trying to bounce the ball
    // on dampen when trying to speed up actions after captures
    const float maxAngleoffBallForDampen = 45; // deg

    // Search settings when trying to find the correct intersection location
    // between the fast moving ball and our collecting robot
    const RJ::Seconds searchStartTime = RJ::Seconds(*_searchStartTime);
    const RJ::Seconds searchEndTime = RJ::Seconds(*_searchEndTime);
    const RJ::Seconds searchIncTime = RJ::Seconds(*_searchIncTime);

    const RJ::Seconds bufferInterceptTime = RJ::Seconds(*_interceptBufferTime);

    const float maxAnglePathTargetChange = *_maxAnglePathTargetChange*M_PI/180.0f;
    const float maxBallAngleChangeForPathReset = *_maxBallAngleForReset*M_PI/180.0f;

    // If the ball changed directions or magnitude really quickly, do a reset of target
    // But if we are already in another state, it should be very quickly to jump out
    // and try again
    if (averageBallVel.angleBetween(ball.vel) > maxBallAngleChangeForPathReset ||
        (averageBallVel - ball.vel).mag() > *_maxBallVelForPathReset) {
        firstTargetPointFound = false;
        firstBallVelFound = false;
    }


    // Smooth out the ball velocity a little bit so we can get a better estimate of intersect points
    if (firstBallVelFound) {
        averageBallVel = *_ballVelGain * ball.vel +
                         (1 - *_ballVelGain) * averageBallVel;
    } else {
        averageBallVel = ball.vel;
        firstBallVelFound = true;
    }

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
        if (timeIntoPreviousPath < prevPath->getDuration() - partialReplanLeadTime &&
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
    if (prevPath && timeIntoPreviousPath < prevPath->getDuration()) {
        Geometry2d::Line ballMovementLine(ball.pos, ball.pos + averageBallVel);
        std::unique_ptr<Path> pathSoFar = prevPath->subPath(0ms, timeIntoPreviousPath);
        
        float botDistToBallMovementLine = ballMovementLine.distTo(pathSoFar->end().motion.pos);

        // Intercept -> Dampen
        //  Almost interescting the ball path
        //    or Almost at end of the target path
        //  Actually in front of the ball
        //
        // TODO: Check ball sense?
        if (timeIntoPreviousPath < prevPath->getDuration() - 1.5*partialReplanLeadTime &&
            (pathSoFar->end().motion.pos - prevPath->end().motion.pos).mag() < 3*Robot_Radius &&
            (pathSoFar->end().motion.pos - ballMovementLine.pt[0]).mag() >
                (pathSoFar->end().motion.pos - ballMovementLine.pt[1]).mag() &&
            currentState == Intercept) {
            
            startInstant = pathSoFar->end().motion;
            currentState = Dampen;
            std::cout << "Transitioned to dampen" << std::endl;
        }
    }

    switch (currentState) {
    case Intercept: {
        // Try find best point to intercept using old method
        // where we check ever X distance along the ball velocity vector
        for (float dist = 0; dist < 5; dist += 0.05) {
            Point ballVelIntercept;
            RJ::Seconds t = RJ::Seconds(ball.estimateTimeTo(ball.pos + averageBallVel.normalized()*dist, &ballVelIntercept) - curTime);
            MotionInstant targetRobotIntersection(ballVelIntercept);
            vector<Point> startEndPoints{startInstant.pos, targetRobotIntersection.pos};

            // TODO: Take the targetFinalCaptureDirection into account
            // Use the mouth to center vector, rotate by X degrees
            // Take the delta between old and new mouth vector and move
            // targetRobotIntersection by that amount

            // TODO: Dodge ball until after reaching intercept point
            targetRobotIntersection.vel = Point(0, 0);

            std::unique_ptr<Path> path =
                RRTPlanner::generatePath(startEndPoints, obstacles, motionConstraints, startInstant.vel, targetRobotIntersection.vel);

            if (path) {
                RJ::Seconds timeOfArrival = path->getDuration();

                // We can reach the target point before 
                if (timeOfArrival + partialPathTime + bufferInterceptTime <= t) {
                    // No path found yet so there is nothing to average
                    if (!firstTargetPointFound) {
                        interceptTarget = targetRobotIntersection.pos;
                        averagePathTime = t;

                        firstTargetPointFound = true;
                    // Average this calculation with the previous ones
                    } else {
                        // Don't allow paths that do not have a continuous velocity state
                        // See issue #1239
                        if (partialPath) {
                            if ((path->start().motion.vel - partialPath->end().motion.vel).mag() > 0.5) {
                                std::cout << "Broken path" << std::endl;
                            }
                        }

                        // Stop any velocity estimation outliers from affecting the path target too much
                        // creating a unachievable path and subsequently causing a large drop in the velocity
                        // at the next state in time
                        // See issue #1239
                        // 
                        // This is will not catch all "bad" paths, but it should get most of them
                        Point robotToNew = targetRobotIntersection.pos - startInstant.pos;
                        Point robotToOld = interceptTarget - startInstant.pos;
                        float angle = robotToNew.angleBetween(robotToOld);

                        if (angle > maxAnglePathTargetChange) {
                            targetRobotIntersection.pos = targetRobotIntersection.pos + 
                                (interceptTarget - targetRobotIntersection.pos).norm() * sin(maxAnglePathTargetChange);
                            std::cout << "Angle too large" << std::endl;
                        }

                        if ((path->start().motion.vel - partialPath->end().motion.vel).mag() > 0.1) {
                            numInvalidPaths++;
                        } else {
                            numInvalidPaths = 0;
                        }

                        if (numInvalidPaths > 5) {
                            std::cout << "X in a row" << std::endl;
                        }

                        interceptTarget = *_targetPointGain * targetRobotIntersection.pos +
                                        (1 - *_targetPointGain) * interceptTarget;

                        averagePathTime = *_targetPointGain * t +
                                        (1 - *_targetPointGain) * averagePathTime;
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
            targetRobotIntersection.vel = *_ballSpeedPercentForDampen * averageBallVel;

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

                // TODO: Predict ball facing location?
                return make_unique<AngleFunctionPath>(
                    std::move(interceptAndDampenStart), angleFunctionForCommandType(
                        FacePointCommand(ball.pos)));
            }
        }

        // No point found
        // Try to move to the closest point in the ball vel line
        Line ballMovementLine(ball.pos, ball.pos + averageBallVel);
        MotionInstant target(ballMovementLine.nearestPoint(startInstant.pos), Point(0,0));

        std::unique_ptr<MotionCommand> rrtCommand =
            std::make_unique<PathTargetCommand>(target);

        auto request = PlanRequest(systemState, startInstant, std::move(rrtCommand),
                                robotConstraints, nullptr, obstacles,
                                dynamicObstacles, planRequest.shellID);
        auto path = rrtPlanner.run(request);
        path->setDebugText("Gives ups");

        std::cout << "Giving up" << std::endl;

        return make_unique<AngleFunctionPath>(
            std::move(path),
            angleFunctionForCommandType(FacePointCommand(ball.pos)));
    }
    case Dampen: {
        // Only run once?

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

        // TODO: Realize the ball will probably bounce off the robot
        // so we can use that vector to stop
        // Save vector and use that?

        if (pathCreatedForDampen && prevPath) {
            return std::move(prevPath);
        }

        pathCreatedForDampen = true;

        if (prevPath) {
            startInstant = prevPath->end().motion;
        }

        // Using the current velocity
        // Calculate stopping point along the ball path
        float maxAccel = motionConstraints.maxAcceleration;
        float currentSpeed = startInstant.vel.mag();

        // Assuming const accel going to zero velocity
        // speed / accel gives time to stop
        // speed / 2 is average time over the entire operation
        float stoppingDist = currentSpeed*currentSpeed / (2 * maxAccel);

        Geometry2d::Point ballMovementDir(averageBallVel.normalized());
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

        if (prevPath) {
            RJ::Time newStartTime = prevPath->startTime();
            dampenEnd = std::make_unique<CompositePath>(std::move(prevPath),
                                                std::move(dampenEnd));
            dampenEnd->setStartTime(newStartTime);
        }

        //std::cout << "New damp path" << std::endl;
        return make_unique<AngleFunctionPath>(
            std::move(dampenEnd),
            angleFunctionForCommandType(FacePointCommand(ball.pos)));
    }
    default: {
        std::cout << "WARNING: Invalid state in settle planner. Restarting" << std::endl;
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

