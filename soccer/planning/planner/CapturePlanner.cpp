#include "CapturePlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "Geometry2d/Pose.hpp"
#include "Geometry2d/Rect.hpp"
#include <functional>

namespace Planning {

using Geometry2d::Point;

REGISTER_CONFIGURABLE(CapturePlanner);

ConfigDouble* CapturePlanner::_searchStartDist;
ConfigDouble* CapturePlanner::_searchEndDist;
ConfigDouble* CapturePlanner::_searchIncDist;
ConfigDouble* CapturePlanner::_maxBallSpeedDirect;
ConfigDouble* CapturePlanner::_maxApproachAngle;
ConfigDouble* CapturePlanner::_maxOutrunBallSpeedPercent;
ConfigDouble* CapturePlanner::_ballContactAccelPercent;
ConfigDouble* CapturePlanner::_bufferDistBeforeContact;
ConfigDouble* CapturePlanner::_bufferDistAfterContact;
ConfigDouble* CapturePlanner::_touchDeltaSpeed;
ConfigDouble* CapturePlanner::_partialReplanLeadTime;
ConfigDouble* CapturePlanner::_ballSpeedPercentForDampen;

void CapturePlanner::createConfiguration(Configuration* cfg) {
    _searchStartDist = new ConfigDouble(cfg,
        "Capture/searchStartDist",0.0);
    _searchEndDist = new ConfigDouble(cfg,
        "Capture/searchEndDist",7.0);
    _searchIncDist = new ConfigDouble(cfg,
        "Capture/searchIncDist",0.2);
    _maxBallSpeedDirect = new ConfigDouble(cfg,
        "Capture/maxBallSpeedDirect", 0.1);
    _maxOutrunBallSpeedPercent = new ConfigDouble(cfg,
        "Capture/maxOutrunBallSpeed", 0.7);
    _maxApproachAngle = new ConfigDouble(cfg,
        "Capture/maxApproachAngle", 0.3);
    _ballContactAccelPercent = new ConfigDouble(cfg,
        "Capture/ballContactAccelPercent", 0.8);
    _bufferDistBeforeContact = new ConfigDouble(cfg,
        "Capture/bufferDistBeforeContact", 0.1);
    _bufferDistAfterContact = new ConfigDouble(cfg,
        "Capture/bufferDistAfterContact", 0.1);
    _touchDeltaSpeed = new ConfigDouble(cfg,
        "Capture/touchDeltaSpeed", 0.2);
    _partialReplanLeadTime = new ConfigDouble(cfg,
        "Capture/partialReplanLeadTime", 0.2);
    _ballSpeedPercentForDampen = new ConfigDouble(cfg,
            "Capture/ballSpeedPercentForDampen", 0);
}

Point CapturePlanner::projectPointIntoField(Point targetPoint, const Geometry2d::Rect& fieldRect, Point ballPoint) const {
    auto intersectReturn = fieldRect.intersects(Geometry2d::Segment(ballPoint, targetPoint));

    bool validIntersect = std::get<0>(intersectReturn);
    std::vector<Point> intersectPts = std::get<1>(intersectReturn);

    // If the ball intersects the field at some point
    // Just get the intersect point as the new target
    if (validIntersect) {
        // Sorts based on distance to intercept target
        // The closest one is the intercept point which the ball moves
        // through leaving the field Not the one on the other side of the
        // field
        // Choose a point just inside the field
        targetPoint = *std::min_element(intersectPts.begin(), intersectPts.end(), [&](Point a, Point b) {
            return (a - targetPoint).mag() <
                   (b - targetPoint).mag();
        });

        // Doesn't intersect
        // project the ball into the field
    } else {
        // Simple projection
        targetPoint.x() = std::clamp(targetPoint.x(),
                                     (double)fieldRect.minx(), (double)fieldRect.maxx());
        targetPoint.y() = std::clamp(targetPoint.y(),
                                     (double)fieldRect.miny(), (double)fieldRect.maxy());
    }
    return targetPoint;
}

Trajectory CapturePlanner::plan(PlanRequest &&request) {
    const Ball &ball = request.context->state.ball;
    CaptureCommand& command = std::get<CaptureCommand>(request.motionCommand);
    RobotConstraints& constraints = request.constraints;
    Trajectory& prevTrajectory = request.prevTrajectory;
    const RJ::Time curTime = RJ::now();

    Point ballToBotUnit = (request.start.pose.position() - ball.pos).norm();
    // the non-negative component of ball.vel pointing toward the robot
    Point ballVelToBot = std::max(0.0, ball.vel.dot(ballToBotUnit)) * ballToBotUnit;
    // the component of ball.vel the robot will have to outrun
    Point awayBallVelocity = ball.vel - ballVelToBot;
    if(awayBallVelocity.mag() > *_maxOutrunBallSpeedPercent * constraints.mot.maxSpeed) {
        debugLog("Impossible Capture Request. The ball is running away too fast.\n");
        return Trajectory{{request.start}};
    }

    // Replan Strategy:
    // - first plan an initial path
    // - try to reuse the previous path unless we find something better
    // - if we'll hit an obstacle soon, partial replan to avoid it
    // - if we're really close to the target, then full replan
    Trajectory partialPath{{}};
    std::optional<Trajectory> reusePath;
    if(!prevTrajectory.empty()) {
        RJ::Seconds timeElapsed = curTime - prevTrajectory.begin_time();
        RJ::Seconds timeRemaining = prevTrajectory.end_time() - curTime;
        RJ::Seconds hitTime;
        bool pathObstructed = prevTrajectory.hit(request.static_obstacles, timeElapsed, &hitTime) || prevTrajectory.intersects(request.dynamic_obstacles, curTime, nullptr, &hitTime);
        if(pathObstructed && hitTime > timeElapsed + RJ::Seconds(2 * *_partialReplanLeadTime)) {
            // the hit is far away so ignore it for now
            pathObstructed = false;
        }
        if(timeRemaining.count() > *_partialReplanLeadTime) {
            // partial replan
            if(prevTrajectory.CheckTime(prevTrajectory.begin_time() + timeElapsed)) {
                partialPath = prevTrajectory.subTrajectory(timeElapsed,
                                                           timeElapsed + RJ::Seconds(*_partialReplanLeadTime));
                request.start = partialPath.last();
            }
            if(!pathObstructed) {
                // reuse previous path unless we find something better
                prevTrajectory.trimFront(timeElapsed);
                reusePath = std::move(prevTrajectory);
            }
        } else if(!veeredOffPath(request)) {
            std::optional<RobotInstant> nowInstant = prevTrajectory.evaluate(curTime);
            if(nowInstant) {
                request.start = *nowInstant;
            }
        }
    }

    Trajectory outputPath{ std::move(partialPath), bruteForceCapture(request)};
    if(!outputPath.empty() && (!reusePath || outputPath.duration() < reusePath->duration())) {
        return std::move(outputPath);
    }
    if(reusePath) {
        return std::move(*reusePath);
    }
    return Trajectory{{request.start}};
}

//todo(motion planning) find a better way to handle moving targets
Trajectory CapturePlanner::bruteForceCapture(const PlanRequest& request) const {
    Ball& ball = request.context->state.ball;
    double distance = *_searchStartDist;
    Point targetPoint = ball.pos;
    int its = 0;
    constexpr RJ::Seconds maxSearchDuration = 6.0s;
    double searchEnd = std::min(_searchEndDist->value(),
                                (ball.predict(ball.time + maxSearchDuration).pos - ball.pos).mag());
    RJ::Time startTime = RJ::now();
    std::optional<Trajectory> path;
    do {
        path = attemptCapture(request, distance);
        if (path) {
            break;
        }
        distance += *_searchIncDist;
        its++;
    } while(distance < searchEnd);
    printf("brute force took %.3f sec, its: %d\n", RJ::Seconds(RJ::now()-startTime).count(), its);
    if(path) {
        return std::move(*path);
    }
    //sometimes RRT fails, so use the old path and try again next iteration
    return std::move(request.prevTrajectory);
}

std::optional<Trajectory> CapturePlanner::attemptCapture(const PlanRequest& request, double distAlongBallLine) const {
    const Ball& ball = request.context->state.ball;
    const CaptureCommand& command = std::get<CaptureCommand>(request.motionCommand);
    const Geometry2d::ShapeSet& static_obstacles = request.static_obstacles;
    const std::vector<DynamicObstacle>& dynamic_obstacles = request.dynamic_obstacles;
    RobotInstant startInstant = request.start;
    const RobotConstraints& constraints = request.constraints;

    const Geometry2d::Rect &fieldRect = Field_Dimensions::Current_Dimensions.FieldRect();
    Point futureBallPoint = ball.pos + ball.vel.normalized(distAlongBallLine);
    bool returnPath = false;
    if (!fieldRect.containsPoint(futureBallPoint)) {
        futureBallPoint = projectPointIntoField(futureBallPoint, fieldRect,
                                                ball.pos);
        returnPath = true;
    }
    RJ::Time futureBallTime = ball.estimateTimeTo(futureBallPoint, &futureBallPoint);

    // collect candidate target states
    // we might try approaching the ball from a few different directions to see
    // which one is faster
    std::vector<Point> contactVels;
    std::vector<double> contactAngles;
    if (ball.vel.mag() > *_maxBallSpeedDirect) {
        if(ball.vel.mag() > *_maxOutrunBallSpeedPercent * constraints.mot.maxSpeed) {
            // the ball is moving fast toward us, so approach opposite ball.vel
            contactVels.push_back(ball.vel * *_ballSpeedPercentForDampen);
            contactAngles.push_back(ball.vel.angle() + M_PI);
        } else {
            // Try both Settle and Collect and pick the quicker one
            contactVels.push_back(ball.vel * *_ballSpeedPercentForDampen);
            contactAngles.push_back(ball.vel.angle() + M_PI);
            contactVels.push_back(ball.vel + ball.vel.normalized(*_touchDeltaSpeed));
            contactAngles.push_back(ball.vel.angle());
        }
    } else {
        double directApproachSpeed = *_touchDeltaSpeed;
        if(command.targetSpeed) {
            directApproachSpeed = *command.targetSpeed;
        }
        contactVels.push_back((ball.pos - startInstant.pose.position()).normalized(directApproachSpeed));
        contactAngles.push_back(startInstant.pose.position().angleTo(ball.pos));
    }
    assert(contactVels.size() == contactAngles.size());

    // Check each candidate target state and pick the trajectory that gets
    // the ball the fastest
    Trajectory outputPath{{}};
    RJ::Time outputPathContactBallTime = RJ::Time::max();
    for(int i = 0; i < contactVels.size(); i++) {
        Point faceDir;
        if(command.targetFacePoint && command.targetSpeed) {
            faceDir = (*command.targetFacePoint - futureBallPoint).norm();
        } else {
            faceDir = Point::direction(contactAngles[i]);
        }

        Point contactPoint = futureBallPoint - (Robot_MouthRadius + Ball_Radius) * faceDir;
        Point contactVel = contactVels[i];
        double contactAngle = faceDir.angle();
        assert(contactVel.mag() <= constraints.mot.maxSpeed);

        if(Point::direction(contactAngle).angleBetween(faceDir) > *_maxApproachAngle) {
            continue;
        }

        // Course: approach the ball in line with the ball velocity
        std::optional<Trajectory> coursePath;
        Point courseTargetPoint = contactPoint - contactVel.normalized(*_bufferDistBeforeContact);
        RobotInstant courseTargetInstant{Geometry2d::Pose{courseTargetPoint, 0}, Geometry2d::Twist{contactVel, 0}, RJ::Time(0s)};
        Point botToContact = contactPoint-startInstant.pose.position();
        // robot is in line with the ball and close to it
        bool inlineWithBall = std::abs(botToContact.cross(contactVel.norm())) < Robot_Radius/2.0;
        bool closeToBall = botToContact.mag() < *_bufferDistBeforeContact;
        RJ::Time contactBallTime = RJ::Time::max();
        //if we aren't in the fine segment
        if(!(inlineWithBall && closeToBall)) {
            coursePath = RRTTrajectory(startInstant, courseTargetInstant, constraints.mot, static_obstacles, dynamic_obstacles);
            if(coursePath->empty()) return std::nullopt;
            courseTargetInstant.stamp = coursePath->last().stamp;
            contactBallTime = coursePath->last().stamp;
//            printf("course target: (%.3f, %.3f, %.3f) (%.3f, %.3f, %.3f) %.6f\n",
//                   courseTargetInstant.pose.position().x(), courseTargetInstant.pose.position().y(), courseTargetInstant.pose.heading(),
//                   courseTargetInstant.velocity.linear().x(), courseTargetInstant.velocity.linear().y(), courseTargetInstant.velocity.angular(),
//                   RJ::Seconds(courseTargetInstant.stamp-coursePath->begin_time()).count()
//            );
        }

        //Fine: constant velocity while contacting the ball
        RobotConstraints fineConstraints = constraints;
        double contactSpeed = contactVel.mag();
        std::optional<Trajectory> finePath;
        // no fine path is needed if we're stationary after course path
        if(contactSpeed > 1e-5) {
            fineConstraints.mot.maxAcceleration *= *_ballContactAccelPercent;
            fineConstraints.mot.maxSpeed = contactSpeed;
            double stoppingDist = std::pow(contactSpeed, 2) / (2 * fineConstraints.mot.maxAcceleration);
            Point fineTargetPoint = contactPoint + faceDir.normalized(*_bufferDistAfterContact + stoppingDist);
            RobotInstant fineTargetInstant{Geometry2d::Pose{fineTargetPoint, 0}, {}, RJ::Time(0s)};
            RobotInstant contactInstant{Geometry2d::Pose{contactPoint, contactAngle}, Geometry2d::Twist{contactVel, 0}, RJ::Time(0s)};
            Trajectory finePathBeforeContact = RRTTrajectory(courseTargetInstant, contactInstant, fineConstraints.mot, static_obstacles, dynamic_obstacles);
            if(finePathBeforeContact.empty()) return std::nullopt;
            contactInstant.stamp = finePathBeforeContact.last().stamp;
            contactBallTime = finePathBeforeContact.last().stamp;
            Trajectory finePathAfterContact = RRTTrajectory(contactInstant, fineTargetInstant, fineConstraints.mot, static_obstacles, dynamic_obstacles);
            if(finePathAfterContact.empty()) return std::nullopt;
            fineTargetInstant.stamp = finePathAfterContact.last().stamp;
            finePath = Trajectory{std::move(finePathBeforeContact), std::move(finePathAfterContact)};
//            printf("contact (%.3f, %.3f, %.3f) (%.3f, %.3f, %.3f) %.6f\n",
//                   contactInstant.pose.position().x(), contactInstant.pose.position().y(), contactInstant.pose.heading(),
//                   contactInstant.velocity.linear().x(), contactInstant.velocity.linear().y(), contactInstant.velocity.angular(),
//                   RJ::Seconds(contactInstant.stamp-finePath->begin_time()).count()
//            );
//            printf("fine target (%.3f, %.3f, %.3f) (%.3f, %.3f, %.3f) %.6f\n",
//                   fineTargetInstant.pose.position().x(), fineTargetInstant.pose.position().y(), fineTargetInstant.pose.heading(),
//                   fineTargetInstant.velocity.linear().x(), fineTargetInstant.velocity.linear().y(), fineTargetInstant.velocity.angular(),
//                   RJ::Seconds(fineTargetInstant.stamp-finePath->begin_time()).count()
//            );
        }

        //combine all the trajectory segments to build the output path
        Trajectory collectPath{{}};
        if(coursePath && finePath) {
            collectPath = Trajectory{std::move(*coursePath), std::move(*finePath)};
        } else if (coursePath) {
            collectPath = std::move(*coursePath);
        } else if (finePath) {
            collectPath = std::move(*finePath);
        }
        if(!collectPath.empty() && (outputPath.empty() || contactBallTime < outputPathContactBallTime)) {
            outputPath = std::move(collectPath);
            outputPathContactBallTime = contactBallTime;
        }
    }
    if(outputPathContactBallTime < futureBallTime) {
        returnPath = true;
    }
    if(returnPath && !outputPath.empty()) {
        return std::move(outputPath);
    }
    return std::nullopt;
}
}