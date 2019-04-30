#include "SettlePathPlanner.hpp"

#include <algorithm>
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
ConfigDouble* SettlePathPlanner::_searchStartDist;
ConfigDouble* SettlePathPlanner::_searchEndDist;
ConfigDouble* SettlePathPlanner::_searchIncDist;
ConfigDouble* SettlePathPlanner::_interceptBufferTime;
ConfigDouble* SettlePathPlanner::_targetPointGain;
ConfigDouble* SettlePathPlanner::_ballVelGain;
ConfigDouble* SettlePathPlanner::_maxBallVelForPathReset;
ConfigDouble* SettlePathPlanner::_maxBallAngleForReset; 

void SettlePathPlanner::createConfiguration(Configuration* cfg) {
    _ballSpeedPercentForDampen =
        new ConfigDouble(cfg, "Capture/Settle/ballSpeedPercentForDampen", 0.1); // %
    _searchStartDist =
        new ConfigDouble(cfg, "Capture/Settle/searchStartDist", 0.0); // m
    _searchEndDist =
        new ConfigDouble(cfg, "Capture/Settle/searchEndDist", 7.0); // m
    _searchIncDist =
        new ConfigDouble(cfg, "Capture/Settle/searchIncDist", 0.2); // m
    _interceptBufferTime =
        new ConfigDouble(cfg, "Capture/Settle/interceptBufferTime", 0.0); // %
    _targetPointGain =
        new ConfigDouble(cfg, "Capture/Settle/targetPointGain", 0.5); // gain between 0 and 1
    _ballVelGain =
        new ConfigDouble(cfg, "Capture/Settle/ballVelGain", 0.5); // gain between 0 and 1
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
    MotionInstant startInstant = planRequest.start;

    // List of obstacles
    ShapeSet& obstacles = planRequest.obstacles;

    // Obstacle list with circle around ball 
    ShapeSet obstaclesWBall = obstacles;
    obstaclesWBall.add(make_shared<Circle>(ball.pos, Robot_Radius+Ball_Radius));

    // Previous RRT path from last iteration
    std::unique_ptr<Path>& prevPath = planRequest.prevPath;

    // Max angle from the ball vector when trying to bounce the ball
    // on dampen when trying to speed up actions after captures
    const float maxAngleoffBallForDampen = 45; // deg

    // Check and see if we should reset the entire thing if we are super far off course
    // or the ball state changes significantly
    checkSolutionValidity(ball, startInstant);


    // Smooth out the ball velocity a little bit so we can get a better estimate of intersect points
    if (firstBallVelFound) {
        averageBallVel = applyLowPassFilter<Point>(averageBallVel, ball.vel, *_ballVelGain);
    } else {
        averageBallVel = ball.vel;
        firstBallVelFound = true;
    }

    // Check if we should transition from intercept to dampen
    // Start instant may be changed in that case since we want to start changing the path
    // as soon as possible
    processStateTransition(ball, prevPath.get(), startInstant);

    // Run state code
    switch (currentState) {
        case Intercept:
            return std::move(intercept(planRequest, curTime, startInstant,
                                       std::move(prevPath), obstaclesWBall));
        case Dampen:
            return std::move(dampen(planRequest, startInstant, std::move(prevPath)));
        default:
            return std::move(invalid(planRequest));
    }
}

void SettlePathPlanner::checkSolutionValidity(const Ball& ball, const MotionInstant& startInstant) {
    const float maxBallAngleChangeForPathReset = *_maxBallAngleForReset*M_PI/180.0f;

    // If the ball changed directions or magnitude really quickly, do a reset of target
    if (averageBallVel.angleBetween(ball.vel) > maxBallAngleChangeForPathReset ||
        (averageBallVel - ball.vel).mag() > *_maxBallVelForPathReset) {
        firstTargetPointFound = false;
        firstBallVelFound = false;
    }

    // Are we too far from the ball line and the ball is still moving
    // or are we too far from a ball not moving towards us
    Geometry2d::Line ballMovementLine(ball.pos, ball.pos + averageBallVel);

    bool robotFar = (ball.pos - startInstant.pos).mag() > 2*Robot_Radius + Ball_Radius;
    bool robotOnBallLine = ballMovementLine.distTo(startInstant.pos) > Robot_MouthWidth;
    bool ballMoving = averageBallVel.mag() > 0.2;
    bool ballMovingToUs = (ball.pos - startInstant.pos).mag() > (ball.pos + 0.01*averageBallVel - startInstant.pos).mag();

    if (((robotOnBallLine && ballMoving) || (robotFar && ballMoving && !ballMovingToUs)) && currentState == Dampen) {
        firstTargetPointFound = false;
        firstBallVelFound = false;

        currentState = Intercept;
    }
}

void SettlePathPlanner::processStateTransition(const Ball& ball,
                                               Path* prevPath,
                                               MotionInstant& startInstant) {
    // State transitions
    // Intercept -> Dampen, PrevPath and almost at the end of the path
    // Dampen -> Complete, PrevPath and almost slowed down to 0?
    if (prevPath && (RJ::now() - prevPath->startTime() > RJ::Seconds(0))) {
        Geometry2d::Line ballMovementLine(ball.pos, ball.pos + averageBallVel);

        const RJ::Seconds timeIntoPreviousPath = RJ::now() - prevPath->startTime();
        std::unique_ptr<Path> pathSoFar = prevPath->subPath(0ms, timeIntoPreviousPath);
        float botDistToBallMovementLine = ballMovementLine.distTo(pathSoFar->end().motion.pos);

        // Intercept -> Dampen
        //  Almost interescting the ball path and
        //  Almost at end of the target path and
        //  Actually in front of the ball
        //
        // TODO: Check ball sense?

        // Within X seconds of the end of path
        bool almostAtEndPath = timeIntoPreviousPath > prevPath->getDuration() - RJ::Seconds(.5);
        bool inlineWithBall =  botDistToBallMovementLine < Robot_Radius;

        if (almostAtEndPath && inlineWithBall && currentState == Intercept) {
            
            // Start the next section of the path from the end of our current path
            startInstant = pathSoFar->end().motion;
            currentState = Dampen;
            std::cout << "Transitioned to dampen" << std::endl;
        }
    }
}

std::unique_ptr<Path> SettlePathPlanner::intercept(const PlanRequest& planRequest,
                                                   const RJ::Time curTime,
                                                   const MotionInstant& startInstant,
                                                   unique_ptr<Path> prevPath,
                                                   const ShapeSet& obstacles) {
    const Ball& ball = planRequest.systemState.ball;

    // Try find best point to intercept using brute force method
    // where we check ever X distance along the ball velocity vector
    // 
    // Disallow points outside the field

    bool foundInterceptPath = false;

    Point ballVelIntercept;
    for (float dist = *_searchStartDist; dist < *_searchEndDist; dist += *_searchIncDist) {
        // Time for ball to reach the target point
        RJ::Seconds ballTime = RJ::Seconds(ball.estimateTimeTo(ball.pos + averageBallVel.normalized()*dist, &ballVelIntercept) - curTime);

        // TODO: Take the targetFinalCaptureDirection into account
        // Use the mouth to center vector, rotate by X degrees
        // Take the delta between old and new mouth vector and move
        // targetRobotIntersection by that amount
        MotionInstant targetRobotIntersection(ballVelIntercept);
        // Should be about stopped at that location
        // Could add a little backwards motion, but it isn't as clean in the planning side
        targetRobotIntersection.vel = Point(0, 0);

        // Plan a path from our partial path start location to the intercept test location
        std::unique_ptr<MotionCommand> rrtCommand =
            std::make_unique<PathTargetCommand>(targetRobotIntersection);

        auto request = PlanRequest(planRequest.systemState,
                                   startInstant,
                                   std::move(rrtCommand),
                                   planRequest.constraints,
                                   nullptr,
                                   obstacles,
                                   planRequest.dynamicObstacles,
                                   planRequest.shellID);

        std::unique_ptr<Path> path = rrtPlanner.run(request);

        // If valid path to location
        // and we can reach the target point before ball
        if (path && path->getDuration() * *_interceptBufferTime <= ballTime) {
            // No path found yet so there is nothing to average
            if (!firstTargetPointFound) {
                interceptTarget = targetRobotIntersection.pos;

                firstTargetPointFound = true;
            // Average this calculation with the previous ones
            } else {                
                interceptTarget = applyLowPassFilter<Point>(interceptTarget,
                                                            targetRobotIntersection.pos,
                                                            *_targetPointGain);
            }

            foundInterceptPath = true;

            break;
        }
    }

    // Could not find a valid path that reach the point first
    // Just go for the farthest point and recalc next time
    if (!foundInterceptPath) {
        if (!firstTargetPointFound) {
            interceptTarget = ballVelIntercept;
        } else {
            interceptTarget = applyLowPassFilter<Point>(interceptTarget, ballVelIntercept, *_targetPointGain);
        }

        std::cout << "Couldn't find valid intercept point" << std::endl;
    }

    // Make sure targetRobotIntersection is inside the field
    // If not, project it into the field
    const Rect& fieldRect =  Field_Dimensions::Current_Dimensions.FieldRect();
    if (!fieldRect.containsPoint(interceptTarget)) {
        auto intersectReturn = fieldRect.intersects(Segment(ball.pos, interceptTarget));

        bool validIntersect = get<0>(intersectReturn);
        vector<Point> intersectPts = get<1>(intersectReturn);

        // If the ball intersects the field at some point
        // Just get the intersect point as the new target
        if (validIntersect) {
            // Sorts based on distance to intercept target
            // The closest one is the intercept point which the ball moves through leaving the field
            // Not the one on the other side of the field
            sort(intersectPts.begin(), intersectPts.end(),
                [&](Point a, Point b) {
                    return (a - interceptTarget).mag() < (b - interceptTarget).mag();
                });

            // Choose a point just inside the field
            interceptTarget = intersectPts.at(0) - Robot_Radius*averageBallVel.norm();

        // Doesn't intersect
        // project the ball into the field
        } else {
            // Simple projection
            interceptTarget.x() = max(interceptTarget.x(), (double)fieldRect.minx() + Robot_Radius);
            interceptTarget.x() = min(interceptTarget.x(), (double)fieldRect.maxx() - Robot_Radius);

            interceptTarget.y() = max(interceptTarget.y(), (double)fieldRect.miny() + Robot_Radius);
            interceptTarget.y() = min(interceptTarget.y(), (double)fieldRect.maxy() - Robot_Radius);
        }
    }

    // Build a new path with the target
    // Since the rrtPlanner exists, we don't have to deal with partial paths, just use the interface
    MotionInstant targetRobotIntersection(interceptTarget);
    targetRobotIntersection.vel = *_ballSpeedPercentForDampen * averageBallVel;

    std::unique_ptr<MotionCommand> rrtCommand =
        std::make_unique<PathTargetCommand>(targetRobotIntersection);

    auto request = PlanRequest(planRequest.systemState,
                                startInstant,
                                std::move(rrtCommand),
                                planRequest.constraints,
                                std::move(prevPath),
                                obstacles,
                                planRequest.dynamicObstacles,
                                planRequest.shellID);

    std::unique_ptr<Path> newTargetPath = rrtPlanner.run(request);

    RJ::Seconds timeOfArrival = newTargetPath->getDuration();
    newTargetPath->setDebugText(QString::number(timeOfArrival.count()) + " s");


    return make_unique<AngleFunctionPath>(
        std::move(newTargetPath),
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
        angleFunctionForCommandType(FacePointCommand(ball.pos - ballMovementDir)));
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
