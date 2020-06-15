#include "SettlePlanner.hpp"

#include <algorithm>
#include <cmath>
#include <planning/TrajectoryUtils.hpp>

#include "Configuration.hpp"
#include "Constants.hpp"
#include "planning/Instant.hpp"
#include "planning/low_level/AnglePlanning.hpp"
#include "planning/low_level/CreatePath.hpp"
#include "planning/low_level/RRTUtil.hpp"

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
    // NOLINTNEXTLINE
    _ballSpeedPercentForDampen = new ConfigDouble(
        cfg, "Capture/Settle/ballSpeedPercentForDampen", 0.1);  // %
    // NOLINTNEXTLINE
    _searchStartDist =
        new ConfigDouble(cfg, "Capture/Settle/searchStartDist", 0.0);  // m
    // NOLINTNEXTLINE
    _searchEndDist =
        new ConfigDouble(cfg, "Capture/Settle/searchEndDist", 7.0);  // m
    // NOLINTNEXTLINE
    _searchIncDist =
        new ConfigDouble(cfg, "Capture/Settle/searchIncDist", 0.2);  // m
    // NOLINTNEXTLINE
    _interceptBufferTime =
        new ConfigDouble(cfg, "Capture/Settle/interceptBufferTime", 0.0);  // %
    // NOLINTNEXTLINE
    _targetPointGain = new ConfigDouble(cfg, "Capture/Settle/targetPointGain",
                                        0.5);  // gain between 0 and 1
    // NOLINTNEXTLINE
    _ballVelGain = new ConfigDouble(cfg, "Capture/Settle/ballVelGain",
                                    0.5);  // gain between 0 and 1
    // NOLINTNEXTLINE
    _shortcutDist = new ConfigDouble(cfg, "Capture/Settle/shortcutDist",
                                     Robot_Radius);  // m
    // NOLINTNEXTLINE
    _maxBallVelForPathReset = new ConfigDouble(
        cfg, "Capture/Settle/maxBallVelForPathReset", 2);  // m/s
    // NOLINTNEXTLINE
    _maxBallAngleForReset = new ConfigDouble(
        cfg, "Capture/Settle/maxBallAngleForReset", 20);  // Deg
    // NOLINTNEXTLINE
    _maxBounceAngle =
        new ConfigDouble(cfg, "Capture/Settle/maxBounceAngle", 45);  // Deg
}

Trajectory SettlePlanner::plan(const PlanRequest& planRequest) {
    BallState ball = planRequest.world_state->ball;

    const RJ::Time curTime = planRequest.start.stamp;

    auto command = std::get<SettleCommand>(planRequest.motionCommand);

    // The direction we will try and bounce the ball when we dampen it to
    // speed up actions after capture
    targetBounceDirection = command.target;

    // Start state for the specified robot
    RobotInstant startInstant = planRequest.start;

    bool avoidBall = true;

    // List of obstacles
    ShapeSet staticObstacles;
    std::vector<DynamicObstacle> dynamicObstacles;
    Trajectory ballTrajectory;
    FillObstacles(planRequest, &staticObstacles, &dynamicObstacles, avoidBall,
                  &ballTrajectory);

    // Smooth out the ball velocity a little bit so we can get a better estimate
    // of intersect points
    if (firstBallVelFound) {
        averageBallVel = applyLowPassFilter<Point>(
            averageBallVel, ball.velocity, *_ballVelGain);
    } else {
        averageBallVel = ball.velocity;
        firstBallVelFound = true;
    }

    // Figure out where we should place the robot and where to face
    // to get the bounce that we want
    // In the case of no input, it defaults to normal behavior
    double angle = startInstant.heading();
    Point deltaPos;
    Point facePos;
    calcDeltaPosForDir(ball, startInstant, &angle, &deltaPos, &facePos);

    // Check and see if we should reset the entire thing if we are super far off
    // course or the ball state changes significantly
    checkSolutionValidity(ball, startInstant, deltaPos);

    if (planRequest.debug_drawer != nullptr) {
        planRequest.debug_drawer->drawLine(
            Segment(ball.position, ball.position + averageBallVel * 10),
            QColor(255, 255, 255), "AverageBallVel");
    }

    // Check if we should transition from intercept to dampen
    // Start instant may be changed in that case since we want to start changing
    // the path as soon as possible
    processStateTransition(ball, &startInstant, angle, deltaPos);

    Trajectory result;

    // Run state code
    switch (currentState) {
        case SettlePlannerStates::Intercept:
            result = intercept(planRequest, startInstant, staticObstacles,
                               dynamicObstacles, deltaPos, facePos);
            break;
        case SettlePlannerStates::Dampen:
            result = dampen(planRequest, startInstant, deltaPos, facePos);
            break;
        default:
            result = invalid(planRequest, staticObstacles, dynamicObstacles);
            break;
    }

    previous = result;
    return result;
}

void SettlePlanner::checkSolutionValidity(BallState ball,
                                          RobotInstant startInstant,
                                          Geometry2d::Point deltaPos) {
    const double maxBallAngleChangeForPathReset =
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
    Point relativeRobotPos = startInstant.position() - deltaPos;

    bool robotFar = (ball.position - relativeRobotPos).mag() >
                    2 * Robot_Radius + Ball_Radius;
    bool robotOnBallLine =
        ballMovementLine.distTo(relativeRobotPos) < Robot_MouthWidth / 2;
    bool ballMoving = averageBallVel.mag() > 0.2;
    bool ballMovingToUs =
        (ball.position - relativeRobotPos).mag() >
        (ball.position + 0.01 * averageBallVel - relativeRobotPos).mag();

    if (((!robotOnBallLine && ballMoving && ballMovingToUs) ||
         (robotFar && ballMoving && !ballMovingToUs)) &&
        currentState == SettlePlannerStates::Dampen) {
        firstInterceptTargetFound = false;
        firstBallVelFound = false;

        currentState = SettlePlannerStates::Intercept;
    }
}

void SettlePlanner::processStateTransition(BallState ball,
                                           RobotInstant* startInstant,
                                           double angle,
                                           Geometry2d::Point deltaPos) {
    // State transitions
    // Intercept -> Dampen, PrevPath and almost at the end of the path
    // Dampen -> Complete, PrevPath and almost slowed down to 0?
    if (!previous.empty() && startInstant->stamp > previous.begin_time() &&
        startInstant->stamp <= previous.end_time()) {
        Geometry2d::Line ballMovementLine(ball.position,
                                          ball.position + averageBallVel);

        Trajectory pathSoFar =
            previous.subTrajectory(previous.begin_time(), startInstant->stamp);
        double botDistToBallMovementLine =
            ballMovementLine.distTo(pathSoFar.last().position() - deltaPos);

        // Intercept -> Dampen
        //  Almost intersecting the ball path and
        //  Almost at end of the target path or
        //  Already in line with the ball
        //
        // TODO(Kyle): Check ball sense?

        // Within X seconds of the end of path
        bool inlineWithBall =
            botDistToBallMovementLine < cos(angle) * Robot_MouthWidth / 2;
        bool inFrontOfBall =
            averageBallVel.angleBetween(startInstant->position() -
                                        ball.position) < 3.14 / 2;

        if (inFrontOfBall && inlineWithBall &&
            currentState == SettlePlannerStates::Intercept) {
            // Start the next section of the path from the end of our current
            // path
            *startInstant = pathSoFar.last();
            currentState = SettlePlannerStates::Dampen;
        }
    }
}

Trajectory SettlePlanner::intercept(
    const PlanRequest& planRequest, RobotInstant startInstant,
    const Geometry2d::ShapeSet& staticObstacles,
    const std::vector<DynamicObstacle>& dynamicObstacles,
    Geometry2d::Point deltaPos, Geometry2d::Point facePos) {
    BallState ball = planRequest.world_state->ball;

    // First, if the previous trajectory is still valid, try to use that.
    if (!previous.empty() && previous.CheckTime(startInstant.stamp) &&
        !TrajectoryHitsStatic(previous, staticObstacles, startInstant.stamp,
                              nullptr) &&
        !TrajectoryHitsDynamic(previous, dynamicObstacles, startInstant.stamp,
                               nullptr, nullptr)) {
        RJ::Seconds timeRemaining = previous.end_time() - startInstant.stamp;
        Point pathAtEnd = previous.last().position();

        Point nearPoint;
        RJ::Seconds timeToIntersect =
            ball.query_seconds_near(pathAtEnd, &nearPoint);
        if (timeRemaining <
                timeToIntersect + RJ::Seconds(*_interceptBufferTime) &&
            nearPoint.distTo(pathAtEnd) < Replanner::goalPosChangeThreshold()) {
            return previous;
        }
    }

    // Try find best point to intercept using brute force method
    // where we check ever X distance along the ball velocity vector
    //
    // Disallow points outside the field
    const Rect& fieldRect = Field_Dimensions::Current_Dimensions.FieldRect();

    Point ballVelIntercept = Geometry2d::Point(0, 0);
    int num_iterations =
        std::ceil((*_searchEndDist - *_searchStartDist) / *_searchIncDist);
    for (int iteration = 0; iteration < num_iterations; iteration++) {
        double dist = *_searchStartDist + iteration * *_searchIncDist;
        // Time for ball to reach the target point
        std::optional<RJ::Seconds> maybeBallTime =
            ball.query_seconds_to_dist(dist);

        if (!maybeBallTime.has_value()) {
            break;
        }

        RJ::Seconds ballTime = maybeBallTime.value();

        // Account for the target point causing a slight offset in robot
        // position since we want the ball to still hit the mouth
        ballVelIntercept =
            ball.position + averageBallVel.normalized() * dist + deltaPos;

        // Use the mouth to center vector, rotate by X degrees
        // Take the delta between old and new mouth vector and move
        // targetRobotIntersection by that amount
        // It should be about stopped at that location.
        // Could add a little backwards motion, but it isn't as clean in the
        // planning side
        LinearMotionInstant targetRobotIntersection{ballVelIntercept, Point()};

        // Plan a path from our partial path start location to the intercept
        // test location
        Replanner::PlanParams params{startInstant,
                                     targetRobotIntersection,
                                     staticObstacles,
                                     dynamicObstacles,
                                     planRequest.constraints,
                                     AngleFns::facePoint(facePos)};
        Trajectory path = Replanner::CreatePlan(params, previous);

        // If valid path to location
        // and we can reach the target point before ball
        //
        // Don't do the average here so we can project the intercept point
        // inside the field
        if (!path.empty() &&
                path.duration() + RJ::Seconds(*_interceptBufferTime) <=
                    ballTime ||
            !fieldRect.containsPoint(ballVelIntercept)) {
            break;
        }
    }

    // Make sure targetRobotIntersection is inside the field
    // If not, project it into the field
    if (!fieldRect.containsPoint(ballVelIntercept)) {
        auto intersectReturn =
            fieldRect.intersects(Segment(ball.position, ballVelIntercept));

        bool validIntersect = std::get<0>(intersectReturn);
        std::vector<Point> intersectPts = std::get<1>(intersectReturn);

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
                std::max(ballVelIntercept.x(), (double)fieldRect.minx());
            ballVelIntercept.x() =
                std::min(ballVelIntercept.x(), (double)fieldRect.maxx());

            ballVelIntercept.y() =
                std::max(ballVelIntercept.y(), (double)fieldRect.miny());
            ballVelIntercept.y() =
                std::min(ballVelIntercept.y(), (double)fieldRect.maxy());
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

    // Shortcuts the crazy path planner to just move into the path of the ball
    // if we are very close Only shortcuts if the target point is further up the
    // path than we are going to hit AKA only shortcut if we have to move
    // backwards along the path to capture the ball
    //
    // Still want to do the math in case the best point changes
    // which happens a lot when the ball is first kicked

    // If we are within a single radius of the ball path
    // and in front of it
    // just move directly to the path location
    Segment ballLine = Segment(
        ball.position, ball.position + averageBallVel.norm() * *_searchEndDist);
    Point closestPt = ballLine.nearestPoint(startInstant.position()) + deltaPos;

    Point ballToPtDir = closestPt - ball.position;
    bool inFrontOfBall = averageBallVel.angleBetween(ballToPtDir) < 3.14 / 2;

    // Only force a direct movement if we are within a small range AND
    // we have run the algorithm at least once AND
    // the target point found in the algorithm is further than we are or just
    // about equal
    if (inFrontOfBall &&
        (closestPt - startInstant.position()).mag() < *_shortcutDist &&
        firstInterceptTargetFound &&
        (closestPt - ball.position).mag() -
                (avgInstantaneousInterceptTarget - ball.position).mag() <
            *_shortcutDist) {
        LinearMotionInstant target{
            closestPt, *_ballSpeedPercentForDampen * averageBallVel};

        Trajectory shortcut = CreatePath::rrt(
            startInstant.linear_motion(), target, planRequest.constraints.mot,
            startInstant.stamp, staticObstacles, dynamicObstacles);

        if (!shortcut.empty()) {
            PlanAngles(&shortcut, startInstant, AngleFns::facePoint(facePos),
                       planRequest.constraints.rot);
            shortcut.stamp(RJ::now());
            return shortcut;
        }
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
    // Since the replanner exists, we don't have to deal with partial paths,
    // just use the interface
    LinearMotionInstant targetRobotIntersection{
        pathInterceptTarget,
        *_ballSpeedPercentForDampen * averageBallVel};

    Replanner::PlanParams params{startInstant,
                                 targetRobotIntersection,
                                 staticObstacles,
                                 dynamicObstacles,
                                 planRequest.constraints,
                                 AngleFns::facePoint(facePos)};
    Trajectory newTargetPath = Replanner::CreatePlan(params, previous);

    RJ::Seconds timeOfArrival = newTargetPath.duration();
    newTargetPath.setDebugText(std::to_string(timeOfArrival.count()) + " s");

    if (newTargetPath.empty()) {
        return previous;
    }

    PlanAngles(&newTargetPath, startInstant, AngleFns::facePoint(facePos),
               planRequest.constraints.rot);
    newTargetPath.stamp(RJ::now());
    return newTargetPath;
}

Trajectory SettlePlanner::dampen(const PlanRequest& planRequest,
                                 RobotInstant startInstant,
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

    // TODO(Kyle): Realize the ball will probably bounce off the robot
    // so we can use that vector to stop
    // Save vector and use that?
    BallState ball = planRequest.world_state->ball;

    if (planRequest.debug_drawer != nullptr) {
        planRequest.debug_drawer->drawText("Damping",
                                           ball.position + Point(.1, .1),
                                           QColor(255, 255, 255), "DampState");
    }

    if (pathCreatedForDampen && !previous.empty()) {
        return previous;
    }

    pathCreatedForDampen = true;

    if (!previous.empty()) {
        startInstant = previous.last();
    }

    // Using the current velocity
    // Calculate stopping point along the ball path
    double maxAccel = planRequest.constraints.mot.maxAcceleration;
    double currentSpeed = startInstant.linear_velocity().mag();

    // Assuming const accel going to zero velocity
    // speed / accel gives time to stop
    // speed / 2 is average time over the entire operation
    double stoppingDist = currentSpeed * currentSpeed / (2 * maxAccel);

    // Offset entire ball line to just be the line we want the robot
    // to move down
    // Accounts for weird targets
    Point ballMovementDir(averageBallVel.normalized());
    Line ballMovementLine(ball.position + deltaPos,
                          ball.position + ballMovementDir + deltaPos);
    Point nearestPointToRobot =
        ballMovementLine.nearestPoint(startInstant.position());
    double distToBallMovementLine =
        (startInstant.position() - nearestPointToRobot).mag();

    // Default to just moving to the closest point on the line
    Point finalStoppingPoint(nearestPointToRobot);

    // Make sure we are actually moving before we start trying to optimize stuff
    if (stoppingDist >= 0.01f) {
        // The closer we are to the line, the less we should move into the line
        // to stop overshoot
        double percentStoppingDistToBallMovementLine =
            distToBallMovementLine / stoppingDist;

        // 0% should be just stopping at stoppingDist down the ball movement
        // line from the nearestPointToRobot 100% or more should just be trying
        // to get to the nearestPointToRobot (Default case)
        if (percentStoppingDistToBallMovementLine < 1) {
            // c^2 - a^2 = b^2
            // c is stopping dist, a is dist to ball line
            // b is dist down ball line
            double distDownBallMovementLine =
                std::sqrt(stoppingDist * stoppingDist -
                          distToBallMovementLine * distToBallMovementLine);
            finalStoppingPoint = nearestPointToRobot +
                                 distDownBallMovementLine * ballMovementDir;
        }
    }

    // Target stopping point with 0 speed.
    LinearMotionInstant finalStoppingMotion{finalStoppingPoint};

    Trajectory dampenEnd =
        CreatePath::simple(startInstant.linear_motion(), finalStoppingMotion,
                           planRequest.constraints.mot, startInstant.stamp);

    dampenEnd.setDebugText("Damping");

    if (!previous.empty()) {
        dampenEnd = Trajectory(previous, dampenEnd);
    }

    PlanAngles(&dampenEnd, startInstant, AngleFns::facePoint(facePos),
               planRequest.constraints.rot);
    dampenEnd.stamp(RJ::now());
    return dampenEnd;
}

Trajectory SettlePlanner::invalid(
    const PlanRequest& planRequest, const Geometry2d::ShapeSet& staticObstacles,
    const std::vector<DynamicObstacle>& dynamicObstacles) {
    std::cout << "WARNING: Invalid state in settle planner. Restarting"
              << std::endl;
    currentState = SettlePlannerStates::Intercept;

    // Stop movement until next frame since it's the safest option
    // programmatically
    LinearMotionInstant target{planRequest.start.position(), Point()};

    Replanner::PlanParams params{
        planRequest.start,
        target,
        staticObstacles,
        dynamicObstacles,
        planRequest.constraints,
        AngleFns::facePoint(planRequest.world_state->ball.position)};
    Trajectory path = Replanner::CreatePlan(params, previous);
    path.setDebugText("Invalid state in settle");
    return path;
}

void SettlePlanner::calcDeltaPosForDir(BallState ball,
                                       RobotInstant startInstant,
                                       double* angle_out,
                                       Geometry2d::Point* deltaRobotPos,
                                       Geometry2d::Point* facePos) {
    // If we have a valid bounce target
    if (targetBounceDirection) {
        // Get angle between target and normal hit
        Point normalFaceVector = ball.position - startInstant.position();
        Point targetFaceVector =
            *targetBounceDirection - startInstant.position();

        // Get the angle between the vectors
        *angle_out = normalFaceVector.angleBetween(targetFaceVector);

        // Clamp so we don't try to bounce behind us
        *angle_out = std::min(*angle_out, (double)*_maxBounceAngle);

        // Since we loose the sign for the angle between call, there are two
        // possibilities
        Point positiveAngle = Point(0, -Robot_MouthRadius * sin(*angle_out))
                                  .rotate(normalFaceVector.angle());
        Point negativeAngle = Point(0, Robot_MouthRadius * sin(*angle_out))
                                  .rotate(normalFaceVector.angle());

        // Choose the closest one to the true angle
        if (targetFaceVector.angleBetween(positiveAngle) <
            targetFaceVector.angleBetween(negativeAngle)) {
            *deltaRobotPos = negativeAngle;
            *facePos =
                startInstant.position() +
                Point::direction(-*angle_out + normalFaceVector.angle()) * 10;
        } else {
            *deltaRobotPos = positiveAngle;
            *facePos =
                startInstant.position() +
                Point::direction(*angle_out + normalFaceVector.angle()) * 10;
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

void SettlePlanner::reset() {
    currentState = SettlePlannerStates::Intercept;
    firstInterceptTargetFound = false;
    firstBallVelFound = false;
    pathCreatedForDampen = false;
    targetBounceDirection = std::nullopt;
    previous = Trajectory{};
}

}  // namespace Planning
