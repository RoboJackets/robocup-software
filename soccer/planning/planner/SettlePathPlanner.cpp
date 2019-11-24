#include "SettlePathPlanner.hpp"

#include <algorithm>
#include <cmath>

#include "Configuration.hpp"
#include "Constants.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/trajectory/PathSmoothing.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"

//todo(Ethan) delete using namespace std
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
ConfigDouble* SettlePathPlanner::_shortcutDist;
ConfigDouble* SettlePathPlanner::_maxBallVelForPathReset;
ConfigDouble* SettlePathPlanner::_maxBallAngleForReset;
ConfigDouble* SettlePathPlanner::_maxBounceAngle;

void SettlePathPlanner::createConfiguration(Configuration* cfg) {
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

Trajectory SettlePathPlanner::plan(PlanRequest&& request) {
    SystemState& systemState = request.context->state;
    const Ball& ball = systemState.ball;

    SettleCommand& command = std::get<SettleCommand>(request.motionCommand);

    // The direction we will try and bounce the ball when we dampen it to
    // speed up actions after capture
    targetBounceDirection = command.target;

    // Start state for the specified robot
    RobotInstant startInstant;
    startInstant.pose = request.start.pose;
    startInstant.velocity = request.start.velocity;
    startInstant.stamp = request.start.stamp;

    // List of obstacles
    ShapeSet& obstacles = request.obstacles;

    // Obstacle list with circle around ball
    ShapeSet obstaclesWBall = obstacles;
    obstaclesWBall.add(
        make_shared<Circle>(ball.pos, Robot_Radius + Ball_Radius));

    // Max angle from the ball vector when trying to bounce the ball
    // on dampen when trying to speed up actions after captures
    const float maxAngleoffBallForDampen = 45;  // deg

    // Smooth out the ball velocity a little bit so we can get a better estimate
    // of intersect points
    if (firstBallVelFound) {
        averageBallVel =
            applyLowPassFilter<Point>(averageBallVel, ball.vel, *_ballVelGain);
    } else {
        averageBallVel = ball.vel;
        firstBallVelFound = true;
    }

    // Figure out where we should place the robot and where to face
    // to get the bounce that we want
    // In the case of no input, it defaults to noraml behavior
    double angle;
    Point deltaPos;
    Point facePos;
    calcDeltaPosForDir(ball, startInstant, angle, deltaPos, facePos);

    // Check and see if we should reset the entire thing if we are super far off
    // course or the ball state changes significantly
    checkSolutionValidity(ball, startInstant, deltaPos);

    request.context->debug_drawer.drawLine(
        Segment(ball.pos, ball.pos + averageBallVel * 10),
        QColor(255, 255, 255), "AverageBallVel");

    // Check if we should transition from intercept to dampen
    // Start instant may be changed in that case since we want to start changing
    // the path as soon as possible
    processStateTransition(ball, request.prevTrajectory, startInstant, angle, deltaPos);

    // Run state code
    switch (currentState) {
        case Intercept:
            return intercept(std::move(request), startInstant, obstaclesWBall, deltaPos, facePos);
        case Dampen:
            return dampen(std::move(request), startInstant, deltaPos, facePos);
        default:
            return invalid(request);
    }
}

void SettlePathPlanner::checkSolutionValidity(const Ball& ball,
                                              const RobotInstant& startInstant,
                                              const Point& deltaPos) {
    const float maxBallAngleChangeForPathReset =
        *_maxBallAngleForReset * M_PI / 180.0f;

    // If the ball changed directions or magnitude really quickly, do a reset of
    // target
    if (averageBallVel.angleBetween(ball.vel) >
            maxBallAngleChangeForPathReset ||
        (averageBallVel - ball.vel).mag() > *_maxBallVelForPathReset) {
        firstInterceptTargetFound = false;
        firstBallVelFound = false;
    }

    // Are we too far from the ball line and the ball is still moving
    // or are we too far from a ball not moving towards us
    Line ballMovementLine(ball.pos, ball.pos + averageBallVel);
    Point relativeRobotPos = startInstant.pose.position() - deltaPos;

    bool robotFar =
        (ball.pos - relativeRobotPos).mag() > 2 * Robot_Radius + Ball_Radius;
    bool robotOnBallLine =
        ballMovementLine.distTo(relativeRobotPos) < Robot_MouthWidth / 2;
    bool ballMoving = averageBallVel.mag() > 0.2;
    bool ballMovingToUs =
        (ball.pos - relativeRobotPos).mag() >
        (ball.pos + 0.01 * averageBallVel - relativeRobotPos).mag();

    if (((!robotOnBallLine && ballMoving && ballMovingToUs) ||
         (robotFar && ballMoving && !ballMovingToUs)) &&
        currentState == Dampen) {
        firstInterceptTargetFound = false;
        firstBallVelFound = false;

        currentState = Intercept;
    }
}

void SettlePathPlanner::processStateTransition(const Ball& ball, const Trajectory& prevTrajectory,
                                               RobotInstant& startInstant,
                                               const double angle,
                                               const Point& deltaPos) {
    // State transitions
    // Intercept -> Dampen, PrevPath and almost at the end of the path
    // Dampen -> Complete, PrevPath and almost slowed down to 0?
    if (!prevTrajectory.empty() && (RJ::now() - prevTrajectory.begin_time() > RJ::Seconds(0)) &&
        prevTrajectory.duration() > RJ::Seconds(0)) {
        Geometry2d::Line ballMovementLine(ball.pos, ball.pos + averageBallVel);

        const RJ::Seconds timeIntoPreviousPath = RJ::now() - prevTrajectory.begin_time();
        Trajectory pathSoFar = prevTrajectory.subTrajectory(0ms, timeIntoPreviousPath);
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
            timeIntoPreviousPath > prevTrajectory.duration() - RJ::Seconds(.5);
        bool inlineWithBall =
            botDistToBallMovementLine < cos(angle) * Robot_MouthWidth / 2;
        bool inFrontOfBall =
            averageBallVel.angleBetween(startInstant.pose.position() - ball.pos) < 3.14 / 2;

        if (inFrontOfBall && inlineWithBall && currentState == Intercept) {
            // Start the next section of the path from the end of our current
            // path
            startInstant = pathSoFar.last();
            currentState = Dampen;
        }
    }
}

Trajectory SettlePathPlanner::intercept(
        PlanRequest&& planRequest, RobotInstant& startInstant,
        const ShapeSet& obstacles, const Point& deltaPos, const Point& facePos) {
    const Ball& ball = planRequest.context->state.ball;

    // Try find best point to intercept using brute force method
    // where we check ever X distance along the ball velocity vector
    //
    // Disallow points outside the field
    const Rect& fieldRect = Field_Dimensions::Current_Dimensions.FieldRect();

    RJ::Time curTime = RJ::now();
    // Use the mouth to center vector, rotate by X degrees
    // Take the delta between old and new mouth vector and move
    // targetRobotIntersection by that amount
    // Should be about stopped at that location
    // Could add a little backwards motion, but it isn't as clean in the
    // planning side
    RobotInstant targetRobotInstersection(Pose(), Twist(), curTime);
    Point& ballVelIntercept = targetRobotInstersection.pose.position();
    for (float dist = *_searchStartDist; dist < *_searchEndDist;
         dist += *_searchIncDist) {
        // Time for ball to reach the target point
        RJ::Seconds ballTime = RJ::Seconds(
            ball.estimateTimeTo(ball.pos + averageBallVel.normalized() * dist,
                                &ballVelIntercept) - curTime);

        // Account for the target point causing a slight offset in robot
        // position since we want the ball to still hit the mouth
        ballVelIntercept += deltaPos;


        // Plan a path from our partial path start location to the intercept
        // test location
        PathTargetCommand pathTargetCommand;
        pathTargetCommand.pathGoal = targetRobotInstersection;

        planRequest.start.pose = startInstant.pose;
        planRequest.start.velocity = startInstant.velocity;
        planRequest.start.stamp = startInstant.stamp;
        Trajectory prevTrajectory = planRequest.prevTrajectory;
        PlanRequest interceptRequest(planRequest.context, planRequest.start, pathTargetCommand,
                                     planRequest.constraints, std::move(prevTrajectory), obstacles, planRequest.shellID);

        Trajectory trajectory = rrtPlanner.plan(std::move(interceptRequest));

        // If valid path to location
        // and we can reach the target point before ball
        //
        // Don't do the average here so we can project the intercept point
        // inside the field
        if (!trajectory.empty() && trajectory.duration() * *_interceptBufferTime <= ballTime ||
            !fieldRect.containsPoint(ballVelIntercept)) {
            break;
        }
    }

    // Make sure targetRobotIntersection is inside the field
    // If not, project it into the field
    if (!fieldRect.containsPoint(ballVelIntercept)) {
        auto intersectReturn =
            fieldRect.intersects(Segment(ball.pos, ballVelIntercept));

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
    Segment ballLine =
        Segment(ball.pos, ball.pos + averageBallVel.norm() * *_searchEndDist);
    Point closestPt = ballLine.nearestPoint(startInstant.pose.position()) + deltaPos;

    Point ballToPtDir = closestPt - ball.pos;
    bool inFrontOfBall = averageBallVel.angleBetween(ballToPtDir) < 3.14 / 2;

    // Only force a direct movement if we are within a small range AND
    // we have run the algorithm at least once AND
    // the target point found in the algorithm is further than we are or just
    // about equal
    if (inFrontOfBall &&
        (closestPt - startInstant.pose.position()).mag() < *_shortcutDist &&
        firstInterceptTargetFound &&
        (closestPt - ball.pos).mag() -
                (avgInstantaneousInterceptTarget - ball.pos).mag() <
            *_shortcutDist) {
        vector<Point> startEnd{startInstant.pose.position(), closestPt};

        Point velFinal = *_ballSpeedPercentForDampen * averageBallVel;
        BezierPath bezier(startEnd, startInstant.velocity.linear(), velFinal, planRequest.constraints.mot);
        Trajectory shortCut = ProfileVelocity(bezier, startInstant.velocity.linear().mag(), velFinal.mag(), planRequest.constraints.mot);
        if(!shortCut.empty()) {
            std::function<double(Point, Point, double)> angleFunction =
                    [](Point pos, Point vel_linear, double angle) -> double {
                        return vel_linear.angle();
                    };
            PlanAngles(shortCut, startInstant, angleFunction, planRequest.constraints.rot);
            shortCut.setDebugText("settle - intercept - short cut");
            return shortCut;
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
    // Since the rrtPlanner exists, we don't have to deal with partial paths,
    // just use the interface
    // todo(Ethan) fix timestamp here; probably shouldn't be RJ::now()
    RobotInstant targetRobotIntersection(
        Pose(pathInterceptTarget, pathInterceptTarget.angleTo(facePos)),
        Twist(*_ballSpeedPercentForDampen * averageBallVel, 0),
        RJ::now());

    PathTargetCommand command;
    command.pathGoal = targetRobotIntersection;

    auto request =
        PlanRequest(planRequest.context, startInstant, command,
                    planRequest.constraints, std::move(planRequest.prevTrajectory), obstacles,
                    planRequest.shellID);

    Trajectory newTargetPath = rrtPlanner.plan(std::move(request));

    RJ::Seconds timeOfArrival = newTargetPath.duration();
    newTargetPath.setDebugText("Intercept " + QString::number(timeOfArrival.count()) + " s");
    return std::move(newTargetPath);
}

Trajectory SettlePathPlanner::dampen(PlanRequest&& planRequest,
                                                RobotInstant& startInstant,
                                                const Point& deltaPos,
                                                const Point& facePos) {
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
    const Ball& ball = planRequest.context->state.ball;
    DebugDrawer& drawer = planRequest.context->debug_drawer;
    planRequest.context->debug_drawer.drawText(
        "Damping", ball.pos + Point(.1, .1), QColor(255, 255, 255),
        "DampState");

    if (pathCreatedForDampen && !planRequest.prevTrajectory.empty()) {
        return std::move(planRequest.prevTrajectory);
    }

    pathCreatedForDampen = true;

    if (!planRequest.prevTrajectory.empty()) {
        startInstant = planRequest.prevTrajectory.last();
    }

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
    Line ballMovementLine(ball.pos + deltaPos,
                          ball.pos + ballMovementDir + deltaPos);
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
    // todo(Ethan) fix this probably should have an actual time stamp instead of RJ::now()
    RobotInstant finalStoppingMotion(Pose(finalStoppingPoint, facePos.angle()), Twist(), RJ::now());
    DirectPathTargetCommand directCommand = DirectPathTargetCommand{finalStoppingMotion};

    auto request =
        PlanRequest(planRequest.context, startInstant, directCommand,
                    planRequest.constraints, Trajectory({}), planRequest.obstacles, planRequest.shellID);

    Trajectory dampenEnd = directPlanner.plan(std::move(request));

    if (!planRequest.prevTrajectory.empty()) {
        dampenEnd = Trajectory(std::move(planRequest.prevTrajectory), std::move(dampenEnd));
    }
    std::function<double(Point,Point,double)> angleFunction =
        [](Point pos, Point vel, double angle) -> double {
            return vel.angle();
        };
    PlanAngles(dampenEnd, startInstant, angleFunction, planRequest.constraints.rot);
    dampenEnd.setDebugText("Damping");
    return std::move(dampenEnd);
}

Trajectory SettlePathPlanner::invalid(
    const PlanRequest& planRequest) {
    std::cout << "WARNING: Invalid state in settle planner. Restarting"
              << std::endl;
    currentState = Intercept;

    // Stop movement until next frame since it's the safest option
    // programmatically
    RobotInstant target(planRequest.start.pose, Twist(), planRequest.start.stamp);

    PathTargetCommand rrtCommand = {target};

    auto request = PlanRequest(
        planRequest.context, planRequest.start, rrtCommand,
        planRequest.constraints, Trajectory({}), planRequest.obstacles, planRequest.shellID);

    auto path = rrtPlanner.plan(std::move(request));
    std::function<double(Point,Point,double)> angleFunction =
            [](Point pos, Point vel, double angle) -> double {
                return vel.angle();
            };
    PlanAngles(path, planRequest.start, angleFunction, planRequest.constraints.rot);
    path.setDebugText("Invalid state in settle");
    return path;
}

void SettlePathPlanner::calcDeltaPosForDir(const Ball& ball,
                                           const RobotInstant& currentInstant,
                                           double& angle,
                                           Geometry2d::Point& deltaRobotPos,
                                           Geometry2d::Point& facePos) {
    // If we have a valid bounce target
    if (targetBounceDirection) {
        // Get angle between target and normal hit
        Point currentPosition = currentInstant.pose.position();
        Point normalFaceVector = ball.pos - currentPosition;
        Point targetFaceVector = *targetBounceDirection - currentPosition;

        // Get the angle between the vectors
        angle = normalFaceVector.angleBetween(targetFaceVector);

        // Clamp so we don't try to bounce behind us
        angle = min(angle, (double)*_maxBounceAngle);

        // Since we loose the sign for the angle between call, there are two
        // possibilities
        Point positiveAngle = Point(0, -Robot_MouthRadius * sin(angle))
                                  .rotate(normalFaceVector.angle());
        Point negativeAngle = Point(0, Robot_MouthRadius * sin(angle))
                                  .rotate(normalFaceVector.angle());

        // Choose the closest one to the true angle
        if (targetFaceVector.angleBetween(positiveAngle) <
            targetFaceVector.angleBetween(negativeAngle)) {
            deltaRobotPos = negativeAngle;
            facePos = currentPosition + Point::direction(-angle + normalFaceVector.angle()) * 10;
        } else {
            deltaRobotPos = positiveAngle;
            facePos = currentPosition + Point::direction(angle + normalFaceVector.angle()) * 10;
        }
    } else {
        deltaRobotPos = Point(0, 0);
        facePos = ball.pos - averageBallVel.normalized();
    }
}

template <typename T>
T SettlePathPlanner::applyLowPassFilter(const T& oldValue, const T& newValue,
                                        double gain) {
    return gain * newValue + (1 - gain) * oldValue;
}
}  // namespace Planning
