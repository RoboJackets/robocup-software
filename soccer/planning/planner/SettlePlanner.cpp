#include "SettlePlanner.hpp"

#include <algorithm>
#include <cmath>

#include "Configuration.hpp"
#include "Constants.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

REGISTER_CONFIGURABLE(SettlePlanner);

ConfigDouble* SettlePlanner::_ballSpeedPercentForDampen;
ConfigDouble* SettlePlanner::_searchStartDist;
ConfigDouble* SettlePlanner::_searchEndDist;
ConfigDouble* SettlePlanner::_searchIncDist;
ConfigDouble* SettlePlanner::_interceptBufferTime;
ConfigDouble* SettlePlanner::_targetPointGain;
ConfigDouble* SettlePlanner::_ballVelGain;
ConfigDouble* SettlePlanner::_shortcutDist;
ConfigDouble* SettlePlanner::_maxBallVelForPathReset;
ConfigDouble* SettlePlanner::_maxBallAngleForReset;
ConfigDouble* SettlePlanner::_maxBounceAngle;

void SettlePlanner::createConfiguration(Configuration* cfg) {
    _ballSpeedPercentForDampen = new ConfigDouble(
        cfg, "Capture/Settle/ballSpeedPercentForDampen", 0.1);  // %
    _searchStartDist =
        new ConfigDouble(cfg, "Capture/Settle/searchStartDist", 0.0);  // m
    _searchEndDist =
        new ConfigDouble(cfg, "Capture/Settle/searchEndDist", 7.0);  // m
    _searchIncDist =
        new ConfigDouble(cfg, "Capture/Settle/searchIncDist", 0.2);  // m
    _interceptBufferTime =
        new ConfigDouble(cfg, "Capture/Settle/interceptBufferTime", 0.0);  // %
    _targetPointGain = new ConfigDouble(cfg, "Capture/Settle/targetPointGain",
                                        0.5);  // gain between 0 and 1
    _ballVelGain = new ConfigDouble(cfg, "Capture/Settle/ballVelGain",
                                    0.5);  // gain between 0 and 1
    _shortcutDist = new ConfigDouble(cfg, "Capture/Settle/shortcutDist",
                                     Robot_Radius);  // m
    _maxBallVelForPathReset = new ConfigDouble(
        cfg, "Capture/Settle/maxBallVelForPathReset", 2);  // m/s
    _maxBallAngleForReset = new ConfigDouble(
        cfg, "Capture/Settle/maxBallAngleForReset", 20);  // Deg
    _maxBounceAngle =
        new ConfigDouble(cfg, "Capture/Settle/maxBounceAngle", 45);  // Deg
}

bool SettlePlanner::shouldReplan(const PlanRequest& planRequest) const {
    return true;
}

Trajectory SettlePlanner::plan(PlanRequest&& planRequest) {
    BallState ball = planRequest.world_state->ball;

    const RJ::Time curTime = RJ::now();

    SettleCommand command = std::get<SettleCommand>(planRequest.motionCommand);

    // The direction we will try and bounce the ball when we dampen it to
    // speed up actions after capture
    targetBounceDirection = command.target;

    // Start state for the specified robot
    RobotInstant startInstant = planRequest.start;

    // List of obstacles
    ShapeSet staticObstacles;
    std::vector<DynamicObstacle> dynamicObstacles;
    Trajectory ballTrajectory;
    FillObstacles(planRequest, &staticObstacles, &dynamicObstacles, true, &ballTrajectory);

    // Max angle from the ball vector when trying to bounce the ball
    // on dampen when trying to speed up actions after captures
    const float maxAngleoffBallForDampen = 45;  // deg

    // Smooth out the ball velocity a little bit so we can get a better estimate
    // of intersect points
    if (firstBallVelFound) {
        averageBallVel =
            applyLowPassFilter<Point>(averageBallVel, ball.velocity, *_ballVelGain);
    } else {
        averageBallVel = ball.velocity;
        firstBallVelFound = true;
    }

    // Figure out where we should place the robot and where to face
    // to get the bounce that we want
    // In the case of no input, it defaults to noraml behavior
    double angle;
    Point deltaPos;
    Point facePos;
    calcDeltaPosForDir(ball, startInstant, &angle, &deltaPos, &facePos);

    // Check and see if we should reset the entire thing if we are super far off
    // course or the ball state changes significantly
    checkSolutionValidity(ball, startInstant, deltaPos);

    if (planRequest.debug_drawer) {
        planRequest.debug_drawer->drawLine(
            Segment(ball.position, ball.position + averageBallVel * 10),
            QColor(255, 255, 255), "AverageBallVel");
    }

    // Check if we should transition from intercept to dampen
    // Start instant may be changed in that case since we want to start changing
    // the path as soon as possible
    processStateTransition(ball, startInstant, angle, deltaPos);

    // Run state code
    Trajectory trajectory;
    switch (currentState) {
        case Intercept:
            trajectory = intercept(planRequest, curTime, startInstant,
                                   staticObstacles, dynamicObstacles,
                                   deltaPos, facePos);
            break;
        case Dampen:
            trajectory = dampen(planRequest, startInstant,
                                staticObstacles, dynamicObstacles,
                                deltaPos, facePos);
            break;
        default:
            trajectory = invalid(planRequest, staticObstacles, dynamicObstacles);
            break;
    }

    previous = trajectory;
    return trajectory;
}

void SettlePlanner::checkSolutionValidity(BallState ball,
                                          RobotInstant startInstant,
                                          Point deltaPos) {
    const float maxBallAngleChangeForPathReset =
        *_maxBallAngleForReset * M_PI / 180.0f;

    // If the ball changed directions or magnitude really quickly, do a reset of
    // target
    if (averageBallVel.angleBetween(ball.velocity) >
        maxBallAngleChangeForPathReset ||
        (averageBallVel - ball.velocity).mag() > *_maxBallVelForPathReset) {
        firstInterceptTargetFound = false;
        firstBallVelFound = false;
    }

    // Are we too far from the ball line and the ball is still moving
    // or are we too far from a ball not moving towards us
    Line ballMovementLine(ball.position, ball.position + averageBallVel);
    Point relativeRobotPos = startInstant.pose.position() - deltaPos;

    bool robotFar =
        (ball.position - relativeRobotPos).mag() > 2 * Robot_Radius + Ball_Radius;
    bool robotOnBallLine =
        ballMovementLine.distTo(relativeRobotPos) < Robot_MouthWidth / 2;
    bool ballMoving = averageBallVel.mag() > 0.2;
    bool ballMovingToUs =
        (ball.position - relativeRobotPos).mag() >
        (ball.position + 0.01 * averageBallVel - relativeRobotPos).mag();

    if (((!robotOnBallLine && ballMoving && ballMovingToUs) ||
         (robotFar && ballMoving && !ballMovingToUs)) &&
        currentState == Dampen) {
        firstInterceptTargetFound = false;
        firstBallVelFound = false;

        currentState = Intercept;
    }
}

void SettlePlanner::processStateTransition(BallState ball,
                                           RobotInstant startInstant,
                                           double angle,
                                           Point deltaPos) {
    // State transitions
    // Intercept -> Dampen, PrevPath and almost at the end of the path
    // Dampen -> Complete, PrevPath and almost slowed down to 0?
    if (!previous.empty() &&
        (RJ::now() - previous.begin_time() > RJ::Seconds(0)) &&
        previous.duration() > RJ::Seconds(0)) {
        Geometry2d::Line ballMovementLine(ball.position, ball.position + averageBallVel);

        const RJ::Seconds timeIntoPreviousPath =
            RJ::now() - previous.begin_time();
        Trajectory pathSoFar =
            previous.subTrajectory(0ms, timeIntoPreviousPath);
        float botDistToBallMovementLine =
            ballMovementLine.distTo(pathSoFar.last().pose.position() - deltaPos);

        // Intercept -> Dampen
        //  Almost interescting the ball path and
        //  Almost at end of the target path or
        //  Already in line with the ball
        //
        // TODO: Check ball sense?

        // Within X seconds of the end of path
        bool almostAtEndPath =
            timeIntoPreviousPath > previous.duration() - RJ::Seconds(.5);
        bool inlineWithBall =
            botDistToBallMovementLine < cos(angle) * Robot_MouthWidth / 2;
        bool inFrontOfBall =
            averageBallVel.angleBetween(startInstant.pose.position() - ball.position) < 3.14 / 2;

        if (inFrontOfBall && inlineWithBall && currentState == Intercept) {
            // Start the next section of the path from the end of our current
            // path
            startInstant = pathSoFar.last();
            currentState = Dampen;
        }
    }
}

Trajectory SettlePlanner::intercept(
    const PlanRequest& planRequest, RJ::Time curTime, RobotInstant startInstant,
    const ShapeSet& staticObstacles,
    const std::vector<DynamicObstacle>& dynamicObstacles,
    Point deltaPos, Point facePos) {
    BallState ball = planRequest.world_state->ball;

    // Try find best point to intercept using brute force method
    // where we check ever X distance along the ball velocity vector
    //
    // Disallow points outside the field
    const Rect& fieldRect = Field_Dimensions::Current_Dimensions.FieldRect();

    Point ballVelIntercept;
    for (float dist = *_searchStartDist; dist < *_searchEndDist;
         dist += *_searchIncDist) {
        // Time for ball to reach the target point
        RJ::Seconds ballTime = ball.query_seconds_to(
            ball.position + averageBallVel.normalized(dist),
            &ballVelIntercept);

        // Account for the target point causing a slight offset in robot
        // position since we want the ball to still hit the mouth
        ballVelIntercept += deltaPos;

        // Use the mouth to center vector, rotate by X degrees
        // Take the delta between old and new mouth vector and move
        // targetRobotIntersection by that amount
        RobotInstant targetRobotIntersection;
        targetRobotIntersection.pose.position() = ballVelIntercept;
        targetRobotIntersection.pose.heading() = startInstant.pose.heading();
        // Should be about stopped at that location
        // Could add a little backwards motion, but it isn't as clean in the
        // planning side
        targetRobotIntersection.velocity.linear() = Point(0, 0);
        targetRobotIntersection.velocity.angular() = 0;

        Replanner::PlanParams params{
            startInstant,
            targetRobotIntersection,
            staticObstacles,
            dynamicObstacles,
            planRequest.constraints,
            AngleFns::facePoint(facePos)
        };
        Trajectory path = rrtPlanner.CreatePlan(params, previous);

        // If valid path to location
        // and we can reach the target point before ball
        //
        // Don't do the average here so we can project the intercept point
        // inside the field
        if (!path.empty() && path.duration() * *_interceptBufferTime <= ballTime ||
            !fieldRect.containsPoint(ballVelIntercept)) {
            break;
        }
    }

    // Make sure targetRobotIntersection is inside the field
    // If not, project it into the field
    if (!fieldRect.containsPoint(ballVelIntercept)) {
        auto intersectReturn =
            fieldRect.intersects(Segment(ball.position, ballVelIntercept));

        bool validIntersect = get<0>(intersectReturn);
        vector<Point> intersectPts = get<1>(intersectReturn);

        // If the ball intersects the field at some point
        // Just get the intersect point as the new target
        if (validIntersect) {
            // Sorts based on distance to intercept target
            // The closest one is the intercept point which the ball moves
            // through leaving the field Not the one on the other side of the
            // field
            sort(intersectPts.begin(), intersectPts.end(),
                 [&](Point a, Point b) {
                   return (a - ballVelIntercept).mag() <
                          (b - ballVelIntercept).mag();
                 });

            // Choose a point just inside the field
            // Add in the deltaPos for weird target angles since the math is
            // not super fun and not really needed
            ballVelIntercept = intersectPts.at(0) + deltaPos;

            // Doesn't intersect
            // project the ball into the field
        } else {
            // Simple projection
            ballVelIntercept.x() =
                max(ballVelIntercept.x(), (double)fieldRect.minx());
            ballVelIntercept.x() =
                min(ballVelIntercept.x(), (double)fieldRect.maxx());

            ballVelIntercept.y() =
                max(ballVelIntercept.y(), (double)fieldRect.miny());
            ballVelIntercept.y() =
                min(ballVelIntercept.y(), (double)fieldRect.maxy());
        }
    }

    // Could not find a valid path that reach the point first
    // Just go for the farthest point and recalc next time
    if (!firstInterceptTargetFound) {
        avgInstantaneousInterceptTarget = ballVelIntercept;
        pathInterceptTarget = ballVelIntercept;

        firstInterceptTargetFound = true;
    } else {
        avgInstantaneousInterceptTarget =
            applyLowPassFilter<Point>(avgInstantaneousInterceptTarget,
                                      ballVelIntercept, *_targetPointGain);
    }

    // There's some major problems with repeatedly changing the target for the
    // path planner To alleviate this problem, we only change the target point
    // when it moves over X amount from the previous path target
    //
    // This combined with the shortcut is guaranteed to get in front of the ball
    // correctly If not, add some sort of distance scale that changes based on
    // how close the robot is to the target
    if ((pathInterceptTarget - avgInstantaneousInterceptTarget).mag() >
        Robot_Radius) {
        pathInterceptTarget = avgInstantaneousInterceptTarget;
    }

    // Build a new path with the target
    // Since the rrtPlanner exists, we don't have to deal with partial paths,
    // just use the interface
    RobotInstant goal;
    goal.pose.position() = pathInterceptTarget;
    goal.pose.heading() = startInstant.pose.heading();
    goal.velocity.linear() = *_ballSpeedPercentForDampen * averageBallVel;

    Replanner::PlanParams params{
        startInstant,
        goal,
        staticObstacles,
        dynamicObstacles,
        planRequest.constraints,
        AngleFns::facePoint(facePos)
    };

    Trajectory trajectory = rrtPlanner.CreatePlan(params, std::move(previous));
    RJ::Seconds timeOfArrival = trajectory.duration();
    trajectory.setDebugText(QString::number(timeOfArrival.count()) + " s");

    return trajectory;
}

Trajectory SettlePlanner::dampen(const PlanRequest& planRequest,
                                 RobotInstant startInstant,
                                 const Geometry2d::ShapeSet& staticObstacles,
                                 const std::vector<DynamicObstacle>& dynamicObstacles,
                                 Geometry2d::Point deltaPos,
                                 Geometry2d::Point facePos) {
    // Only run once if we can

    // Intercept ends with a % ball velocity in the direction of the ball
    // movement Slow down once ball is nearby to 0 m/s

    // Try to slow down as fast as possible to 0 m/s along the ball path
    // We have to do position control since we want to stay in the line of the
    // ball while we do this. If we did velocity, we have very little control of
    // where on the field it is without some other position controller.

    // Uses constant acceleration to create a linear velocity profile

    // TODO: Realize the ball will probably bounce off the robot
    // so we can use that vector to stop
    // Save vector and use that?
    BallState ball = planRequest.world_state->ball;

    if (planRequest.debug_drawer != nullptr) {
        planRequest.debug_drawer->drawText(
            "Damping", ball.position + Point(.1, .1), QColor(255, 255, 255),
            "DampState");
    }

    if (pathCreatedForDampen && !previous.empty()) {
        return previous;
    }

    pathCreatedForDampen = true;

    // Using the current velocity
    // Calculate stopping point along the ball path
    float maxAccel = planRequest.constraints.mot.maxAcceleration;
    float currentSpeed = startInstant.velocity.linear().mag();

    // Assuming const accel going to zero velocity
    // speed / accel gives time to stop
    // speed / 2 is average time over the entire operation
    float stoppingDist = currentSpeed * currentSpeed / (2 * maxAccel);

    // Offset entire ball line to just be the line we want the robot
    // to move down
    // Accounts for weird targets
    Point ballMovementDir(averageBallVel.normalized());
    Line ballMovementLine(ball.position + deltaPos,
                          ball.position + ballMovementDir + deltaPos);
    Point nearestPointToRobot = ballMovementLine.nearestPoint(startInstant.pose.position());
    float distToBallMovementLine =
        (startInstant.pose.position() - nearestPointToRobot).mag();

    // Default to just moving to the closest point on the line
    Point finalStoppingPoint(nearestPointToRobot);

    // Make sure we are actually moving before we start trying to optimize stuff
    if (stoppingDist >= 0.01f) {
        // The closer we are to the line, the less we should move into the line
        // to stop overshoot
        float percentStoppingDistToBallMovementLine =
            distToBallMovementLine / stoppingDist;

        // 0% should be just stopping at stoppingDist down the ball movement
        // line from the nearestPointToRobot 100% or more should just be trying
        // to get to the nearestPointToRobot (Default case)
        if (percentStoppingDistToBallMovementLine < 1) {
            // c^2 - a^2 = b^2
            // c is stopping dist, a is dist to ball line
            // b is dist down ball line
            float distDownBallMovementLine =
                sqrt(stoppingDist * stoppingDist -
                     distToBallMovementLine * distToBallMovementLine);
            finalStoppingPoint = nearestPointToRobot +
                                 distDownBallMovementLine * ballMovementDir;
        }
    }

    // Target stopping point with 0 speed.
    RobotInstant finalStoppingMotion;
    finalStoppingMotion.pose.position() = finalStoppingPoint;
    finalStoppingMotion.pose.heading() = startInstant.pose.heading();

    Replanner::PlanParams params{
        startInstant,
        finalStoppingMotion,
        staticObstacles,
        dynamicObstacles,
        planRequest.constraints,
        AngleFns::facePoint(facePos)
    };

    Trajectory trajectory = rrtPlanner.CreatePlan(params,
                                                  std::move(previous));
    trajectory.setDebugText("Damping");

    return trajectory;
}

Trajectory SettlePlanner::invalid(
    const PlanRequest& planRequest,
    const ShapeSet& staticObstacles,
    const std::vector<DynamicObstacle>& dynamicObstacles) {
    std::cout << "WARNING: Invalid state in collect planner. Restarting"
              << std::endl;
    currentState = Intercept;

    // Stop movement until next frame since it's the safest option
    // programmatically
    RobotInstant target(planRequest.start);
    target.velocity = Twist();

    Replanner::PlanParams params{
        planRequest.start,
        target,
        staticObstacles,
        dynamicObstacles,
        planRequest.constraints,
        AngleFns::facePoint(planRequest.world_state->ball.position)
    };

    Trajectory path = rrtPlanner.CreatePlan(params,
                                            std::move(previous));
    path.setDebugText("Invalid state in collect");

    return path;
}

void SettlePlanner::calcDeltaPosForDir(BallState ball,
                                       RobotInstant startInstant,
                                       double* angle,
                                       Geometry2d::Point* deltaRobotPos,
                                       Geometry2d::Point* facePos) {
    // If we have a valid bounce target
    if (targetBounceDirection) {
        // Get angle between target and normal hit
        Point normalFaceVector = ball.position - startInstant.pose.position();
        Point targetFaceVector = *targetBounceDirection - startInstant.pose.position();

        // Get the angle between the vectors
        *angle = normalFaceVector.angleBetween(targetFaceVector);

        // Clamp so we don't try to bounce behind us
        *angle = std::min(*angle, (double)*_maxBounceAngle);

        // Since we loose the sign for the angle between call, there are two
        // possibilities
        Point positiveAngle = Point(0, -Robot_MouthRadius * std::sin(*angle))
            .rotate(normalFaceVector.angle());
        Point negativeAngle = Point(0, Robot_MouthRadius * std::sin(*angle))
            .rotate(normalFaceVector.angle());

        // Choose the closest one to the true angle
        if (targetFaceVector.angleBetween(positiveAngle) <
            targetFaceVector.angleBetween(negativeAngle)) {
            *deltaRobotPos = negativeAngle;
            *facePos = startInstant.pose.position() +
                      Point::direction(-*angle + normalFaceVector.angle()) * 10;
        } else {
            *deltaRobotPos = positiveAngle;
            *facePos = startInstant.pose.position() +
                      Point::direction(*angle + normalFaceVector.angle()) * 10;
        }
    } else {
        *deltaRobotPos = Point(0, 0);
        *facePos = ball.position - averageBallVel.normalized();
    }
}

template <typename T>
T SettlePlanner::applyLowPassFilter(const T& oldValue, const T& newValue,
                                        double gain) {
    return gain * newValue + (1 - gain) * oldValue;
}
}  // namespace Planning
