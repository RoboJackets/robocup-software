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
ConfigDouble* SettlePathPlanner::_ballAvoidDistance;
ConfigDouble* SettlePathPlanner::_interceptBufferTime;
ConfigDouble* SettlePathPlanner::_targetPointGain;
ConfigDouble* SettlePathPlanner::_ballVelGain;
ConfigInt*    SettlePathPlanner::_maxNumInvalidPaths;
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
    _ballAvoidDistance =
        new ConfigDouble(cfg, "Capture/Settle/ballAvoidDistance", 0.3); // m
    _interceptBufferTime =
        new ConfigDouble(cfg, "Capture/Settle/interceptBufferTime", 0.0); // Seconds
    _targetPointGain =
        new ConfigDouble(cfg, "Capture/Settle/targetPointGain", 0.5); // gain between 0 and 1
    _ballVelGain =
        new ConfigDouble(cfg, "Capture/Settle/ballVelGain", 0.5); // gain between 0 and 1
    _maxNumInvalidPaths =
        new ConfigInt(cfg, "Capture/Settle/maxNumInvalidPaths", 3); // int
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

    // List of obstacles
    ShapeSet& obstacles = planRequest.obstacles;

    // Obstacle list with circle around ball 
    ShapeSet obstaclesWBall = obstacles;
    obstaclesWBall.add(make_shared<Circle>(ball.pos, *_ballAvoidDistance));

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


    // Max angle from the ball vector when trying to bounce the ball
    // on dampen when trying to speed up actions after captures
    const float maxAngleoffBallForDampen = 45; // deg

    // Check and see if we should reset the entire thing if we are super far off course
    // or the ball state changes significantly
    checkSolutionValidity(ball);


    // Smooth out the ball velocity a little bit so we can get a better estimate of intersect points
    if (firstBallVelFound) {
        averageBallVel = applyLowPassFilter<Point>(averageBallVel, ball.vel, *_ballVelGain);
    } else {
        averageBallVel = ball.vel;
        firstBallVelFound = true;
    }

    // Change start instant to be the partial path end instead of the robot current location
    // if we actually have already calculated a path the frame before
    if (prevPath) {
        // How much of the previous path to steal
        const RJ::Seconds partialReplanLeadTime = RRTPlanner::getPartialReplanLeadTime();

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

    // Check if we should transition from intercept to dampen
    // Start instant may be changed in that case since we want to start changing the path
    // as soon as possible
    processStateTransition(ball, prevPath.get(), timeIntoPreviousPath, startInstant);

    // Run state code
    // TODO: Do I need to do a move or is it implicit?
    switch (currentState) {
        case Intercept:
            return std::move(intercept(planRequest, curTime, startInstant,
                                       std::move(partialPath), partialPathTime,
                                       obstaclesWBall));
        case Dampen:
            return std::move(dampen(planRequest, startInstant, std::move(prevPath)));
        default:
            return std::move(invalid(planRequest));
    }
}

void SettlePathPlanner::checkSolutionValidity(const Ball& ball) {
    const float maxBallAngleChangeForPathReset = *_maxBallAngleForReset*M_PI/180.0f;

    // If the ball changed directions or magnitude really quickly, do a reset of target
    // But if we are already in another state, it should be very quickly to jump out
    // and try again
    if (averageBallVel.angleBetween(ball.vel) > maxBallAngleChangeForPathReset ||
        (averageBallVel - ball.vel).mag() > *_maxBallVelForPathReset) {
        firstTargetPointFound = false;
        firstBallVelFound = false;
    }
}

void SettlePathPlanner::processStateTransition(const Ball& ball,
                                               Path* prevPath,
                                               const RJ::Seconds& timeIntoPreviousPath,
                                               MotionInstant& startInstant) {
    // How much of the previous path to steal
    const RJ::Seconds partialReplanLeadTime = RRTPlanner::getPartialReplanLeadTime();

    // State transitions
    // Intercept -> Dampen, PrevPath and almost at the end of the path
    // Dampen -> Complete, PrevPath and almost slowed down to 0?
    if (prevPath && timeIntoPreviousPath < prevPath->getDuration()) {
        Geometry2d::Line ballMovementLine(ball.pos, ball.pos + averageBallVel);
        std::unique_ptr<Path> pathSoFar = prevPath->subPath(0ms, timeIntoPreviousPath);
        
        float botDistToBallMovementLine = ballMovementLine.distTo(pathSoFar->end().motion.pos);

        // Intercept -> Dampen
        //  Almost interescting the ball path and
        //  Almost at end of the target path and
        //  Actually in front of the ball
        //
        // TODO: Check ball sense?
        if (timeIntoPreviousPath < prevPath->getDuration() - 1.5*partialReplanLeadTime &&
            (pathSoFar->end().motion.pos - prevPath->end().motion.pos).mag() < 3*Robot_Radius &&
            (pathSoFar->end().motion.pos - ballMovementLine.pt[0]).mag() >
                (pathSoFar->end().motion.pos - ballMovementLine.pt[1]).mag() &&
            currentState == Intercept) {
            
            // TODO: Is this what I really want?
            startInstant = pathSoFar->end().motion;
            currentState = Dampen;
            std::cout << "Transitioned to dampen" << std::endl;
        }
    }
}

std::unique_ptr<Path> SettlePathPlanner::intercept(const PlanRequest& planRequest,
                                                   const RJ::Time curTime,
                                                   const MotionInstant& startInstant,
                                                   unique_ptr<Path> partialPath,
                                                   const RJ::Seconds partialPathTime,
                                                   const ShapeSet& obstacles) {
    const Ball& ball = planRequest.systemState.ball;

    // Search settings when trying to find the correct intersection location
    // between the fast moving ball and our collecting robot
    // TODO: Either switch to a time or change config to dist
    const RJ::Seconds searchStartTime = RJ::Seconds(*_searchStartTime);
    const RJ::Seconds searchEndTime = RJ::Seconds(*_searchEndTime);
    const RJ::Seconds searchIncTime = RJ::Seconds(*_searchIncTime);

    // How much earlier should we be at the point compaired to the ball
    const RJ::Seconds bufferInterceptTime = RJ::Seconds(*_interceptBufferTime);

    bool foundInterceptPath = false;

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

        ShapeSet obstacleWEnd = obstacles;
        obstacleWEnd.add(make_shared<Circle>(ball.pos + averageBallVel.normalized()*dist, *_ballAvoidDistance));

        // TODO: Dodge ball until after reaching intercept point
        targetRobotIntersection.vel = Point(0, 0);

        std::unique_ptr<Path> path = RRTPlanner::generatePath(startEndPoints,
                                                              obstacleWEnd,
                                                              planRequest.constraints.mot,
                                                              startInstant.vel,
                                                              targetRobotIntersection.vel);

        // If valid path to location
        // and we can reach the target point before ball
        if (path && path->getDuration() + partialPathTime + bufferInterceptTime <= t) {
            // No path found yet so there is nothing to average
            if (!firstTargetPointFound) {
                interceptTarget = targetRobotIntersection.pos;
                averagePathTime = t;

                firstTargetPointFound = true;
            // Average this calculation with the previous ones
            } else {
                // Stop any velocity estimation outliers from affecting the path target too much
                // creating a unachievable path and subsequently causing a large drop in the velocity
                // at the next state in time
                // See issue #1239
                // 
                // This happens quite a bit at the beginning of a path due to the not fully correct
                // estimate of the ball velocity. Once it stabilizes, the path usually doesn't change
                //
                // This allows at most 3 invalid paths in a row before deciding that we need to change
                //
                // Gotta make sure that this fails only when there is a discontinuity in the path,
                // not when partial path is invalid
                bool invalidPath = (partialPath && (path->start().motion.vel - partialPath->end().motion.vel).mag() > 0.01) ||
                                    !partialPath;

                // if (valid path) OR (3 invalid paths in a row)
                // Use the current path and average in
                //
                // Increments the number of invalid path's if it's not valid
                // If we hit that limit, we should change the path
                // Always reset the number of invalid paths when a new value is averaged in
                if (invalidPath) {
                    numInvalidPaths++;
                }

                if (!invalidPath || numInvalidPaths > *_maxNumInvalidPaths) {
                    interceptTarget = applyLowPassFilter<Point>(interceptTarget,
                                                                targetRobotIntersection.pos,
                                                                *_targetPointGain);

                    averagePathTime = applyLowPassFilter<RJ::Seconds>(averagePathTime, t, *_targetPointGain);

                    // Debug print
                    // TODO: Remove
                    if (numInvalidPaths > *_maxNumInvalidPaths) {
                        std::cout << "Changing paths" << std::endl;
                    }

                    numInvalidPaths = 0;
                }
            }

            foundInterceptPath = true;

            break;
        }
    }

    // Could not find a valid path that reach the point first
    // Just try and hit the closest point in velocity line
    if (!foundInterceptPath) {
        Line ballMovementLine(ball.pos, ball.pos + averageBallVel);
        interceptTarget = ballMovementLine.nearestPoint(startInstant.pos);

        // If we are trying to intercept the line behind the ball, just go for the position of the ball
        // 1 second from now
        //if ((interceptTarget - ball.pos).mag() < (interceptTarget - (ball.pos + averageBallVel)).mag()) {
        //    interceptTarget = ball.pos + averageBallVel;
        //    std::cout << "giving up in front" << std::endl;
        //}
    }

    // Try and use the previous path for the first part so it will actually make the initial turn
    MotionInstant targetRobotIntersection(interceptTarget);
    std::vector<Geometry2d::Point> startEndPoints{startInstant.pos, targetRobotIntersection.pos};
    targetRobotIntersection.vel = *_ballSpeedPercentForDampen * averageBallVel;

    std::unique_ptr<Path> interceptAndDampenStart = RRTPlanner::generatePath(startEndPoints,
                                                                             obstacles,
                                                                             planRequest.constraints.mot,
                                                                             startInstant.vel,
                                                                             targetRobotIntersection.vel);

    if (interceptAndDampenStart) {
        RJ::Seconds timeOfArrival = interceptAndDampenStart->getDuration();
        interceptAndDampenStart->setDebugText(QString::number(timeOfArrival.count()) + " s");

        if (partialPath) {
            // You have to create a temp variable otherwise partialPath goes to nullptr after the
            // composite path creation
            RJ::Time startTime = partialPath->startTime();

            interceptAndDampenStart = std::make_unique<CompositePath>(std::move(partialPath),
                                                                      std::move(interceptAndDampenStart));
            interceptAndDampenStart->setStartTime(startTime);
        }

        return make_unique<AngleFunctionPath>(
            std::move(interceptAndDampenStart),
            angleFunctionForCommandType(FacePointCommand(ball.pos)));
    }

    // Path failed, just try a different way. I don't think it's possible for this one to fail?
    // Not sure why this is needed, but it's used in line kick
    MotionInstant target(interceptTarget, Point(0,0));

    std::unique_ptr<MotionCommand> rrtCommand =
        std::make_unique<PathTargetCommand>(target);

    auto request = PlanRequest(planRequest.systemState,
                               planRequest.start,
                               std::move(rrtCommand),
                               planRequest.constraints,
                               nullptr,
                               obstacles,
                               planRequest.dynamicObstacles,
                               planRequest.shellID);

    auto path = rrtPlanner.run(request);
    path->setDebugText("Gives ups");

    std::cout << "Giving up" << std::endl;

    return make_unique<AngleFunctionPath>(
        std::move(path),
        angleFunctionForCommandType(FacePointCommand(ball.pos)));
}

std::unique_ptr<Path> SettlePathPlanner::dampen(const PlanRequest& planRequest,
                                                MotionInstant& startInstant,
                                                unique_ptr<Path> prevPath) {
    // Only run once if we can

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
    const Ball& ball = planRequest.systemState.ball;

    if (pathCreatedForDampen && prevPath) {
        return std::move(prevPath);
    }

    pathCreatedForDampen = true;

    if (prevPath) {
        startInstant = prevPath->end().motion;
    }

    // Using the current velocity
    // Calculate stopping point along the ball path
    float maxAccel = planRequest.constraints.mot.maxAcceleration;
    float currentSpeed = startInstant.vel.mag();

    // Assuming const accel going to zero velocity
    // speed / accel gives time to stop
    // speed / 2 is average time over the entire operation
    float stoppingDist = currentSpeed*currentSpeed / (2 * maxAccel);

    Point ballMovementDir(averageBallVel.normalized());
    Line ballMovementLine(ball.pos, ball.pos + ballMovementDir);
    Point nearestPointToRobot = ballMovementLine.nearestPoint(startInstant.pos);
    float distToBallMovementLine = (startInstant.pos - nearestPointToRobot).mag();

    // Default to just moving to the closest point on the line
    Point finalStoppingPoint(nearestPointToRobot);

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
    MotionInstant finalStoppingMotion(finalStoppingPoint, Point(0,0));
    std:unique_ptr<MotionCommand> rrtCommand = 
        std::make_unique<DirectPathTargetCommand>(finalStoppingMotion);

    auto request = PlanRequest(planRequest.systemState,
                               startInstant,
                               std::move(rrtCommand),
                               planRequest.constraints,
                               nullptr,
                               planRequest.obstacles,
                               planRequest.dynamicObstacles,
                               planRequest.shellID);

    std::unique_ptr<Path> dampenEnd = directPlanner.run(request);
    dampenEnd->setDebugText("Damping");

    if (prevPath) {
        RJ::Time newStartTime = prevPath->startTime();
        dampenEnd = std::make_unique<CompositePath>(std::move(prevPath),
                                                    std::move(dampenEnd));
        dampenEnd->setStartTime(newStartTime);
    }

    return make_unique<AngleFunctionPath>(
        std::move(dampenEnd),
        angleFunctionForCommandType(FacePointCommand(ball.pos)));
}

std::unique_ptr<Path> SettlePathPlanner::invalid(const PlanRequest& planRequest) {
    std::cout << "WARNING: Invalid state in settle planner. Restarting" << std::endl;
    currentState = Intercept;

    // Stop movement until next frame since it's the safest option programmatically
    MotionInstant target(planRequest.start);
    target.vel = Point(0, 0);

    std::unique_ptr<MotionCommand> rrtCommand =
        std::make_unique<PathTargetCommand>(target);

    auto request = PlanRequest(planRequest.systemState,
                               planRequest.start,
                               std::move(rrtCommand),
                               planRequest.constraints,
                               nullptr,
                               planRequest.obstacles,
                               planRequest.dynamicObstacles,
                               planRequest.shellID);

    auto path = rrtPlanner.run(request);
    path->setDebugText("Invalid state in settle");

    return make_unique<AngleFunctionPath>(
        std::move(path),
        angleFunctionForCommandType(FacePointCommand(planRequest.systemState.ball.pos)));
}

template<typename T>
T SettlePathPlanner::applyLowPassFilter(const T& oldValue, const T& newValue, double gain) {
    return gain*newValue + (1 - gain)*oldValue;
}
}
