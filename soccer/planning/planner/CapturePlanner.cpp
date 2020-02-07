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

    Point ballToBot = request.start.pose.position() - ball.pos;
    Point ballToBotUnit = ballToBot.norm();
    // the non-negative component of ball.vel pointing toward the robot
    Point ballVelToBot = std::max(0.0, ball.vel.dot(ballToBotUnit)) * ballToBotUnit;
    // the component of ball.vel the robot will have to outrun
    Point awayBallVelocity = ball.vel - ballVelToBot;
    if(awayBallVelocity.mag() > *_maxOutrunBallSpeedPercent * constraints.mot.maxSpeed) {
        //Impossible Capture Request. The ball is running away too fast
        //Todo(Ethan) handle this probably
    }

//    // if the ball changed course, the old trajectory is invalid todo(Ethan) uncomment? delete?
//    if(!prevBall) prevBall = ball;
//    MotionInstant oldBallInstant = prevBall->predict(ball.time);
//    double ballPosChange = ball.pos.distTo(oldBallInstant.pos);
//    double ballVelAngleChange = ball.vel.angleBetween(oldBallInstant.vel);
//    bool ballChangedCourse = ballPosChange > maxBallPosChange || ballVelAngleChange > maxBallVelAngleChange;
//    prevBall = ball;
//    if(ballChangedCourse) prevTrajectory = Trajectory{{}};
//
//    if(!prevTrajectory.CheckTime(RJ::now())) prevTrajectory = Trajectory{{}};

    Trajectory partialPath{{}};
    bool fullReplan = false;
    bool partialReplan = false;
    bool reusePath = false;
    RJ::Seconds timeElapsed = curTime - prevTrajectory.begin_time();
    if(veeredOffPath(request) || prevTrajectory.empty()) {
        fullReplan = true;
    } else {
        RJ::Seconds timeRemaining = prevTrajectory.end_time() - curTime;
        RJ::Seconds invalidTime;
        partialReplan = prevTrajectory.hit(request.static_obstacles, timeElapsed, &invalidTime) || prevTrajectory.intersects(request.dynamic_obstacles, curTime, nullptr, &invalidTime);
        Point prevGoalPos = prevTrajectory.last().pose.position();
        Point curGoalPos = ball.predict(std::max(RJ::now(), prevTrajectory.end_time())).pos;
        bool goalChanged = prevGoalPos.distTo(curGoalPos) > 0.3; //todo(Ethan) no magic numbers
        if(!partialReplan && goalChanged) {
            partialReplan = true;
            invalidTime = prevTrajectory.duration();
        }
        if(partialReplan && invalidTime < timeElapsed + RJ::Seconds(2 * *_partialReplanLeadTime)) {
            fullReplan = true;
        }
    }
    if(fullReplan) {
        partialPath = Trajectory{{request.start}};
    } else if(partialReplan) {
        partialPath = prevTrajectory.subTrajectory(timeElapsed,
                                                   timeElapsed + RJ::Seconds(*_partialReplanLeadTime));
        request.start = partialPath.last();
    } else {
        return reuse(std::move(request));
    }
    std::optional<Trajectory> brutePath = bruteForceCapture(request);
    if(!brutePath) {
        if(!prevTrajectory.empty()) {
            return reuse(std::move(request));
        } else {
            return Trajectory{{request.start}};
        }
    }
    return Trajectory{std::move(partialPath), std::move(*brutePath)};
}

//todo(motion planning) find a better way to handle moving targets
std::optional<Trajectory> CapturePlanner::bruteForceCapture(const PlanRequest& request) const {
    Ball& ball = request.context->state.ball;
    int its = 0;
    constexpr RJ::Seconds maxSearchDuration = 1.5s;
    std::optional<Trajectory> path;
    RJ::Time startTime = RJ::now();
    RJ::Time contactTime = startTime;

    //todo(Ethan) fix
    return attemptCapture(request, startTime);
    while(contactTime < startTime + maxSearchDuration) {
        std::optional<Trajectory> candidatePath = attemptCapture(request, contactTime);
        if (candidatePath && !candidatePath->empty() && (!path || candidatePath->duration() < path->duration())) {
            path = candidatePath;
        }
        contactTime = RJ::Time{contactTime + RJ::Seconds{0.15}};
        its++;
    }
    printf("brute force took %.3f sec, its: %d\n", RJ::Seconds(RJ::now()-startTime).count(), its);
    if(path) {
        assert(!path->empty());
        return std::move(*path);
    }
    //sometimes RRT fails, so use the old path and try again next iteration
    return std::nullopt;
}

std::optional<Trajectory> CapturePlanner::attemptCapture(const PlanRequest& request, RJ::Time contactTime) const {
    const Ball& ball = request.context->state.ball;
    const CaptureCommand& command = std::get<CaptureCommand>(request.motionCommand);
    const Geometry2d::ShapeSet& static_obstacles = request.static_obstacles;
    const std::vector<DynamicObstacle>& dynamic_obstacles = request.dynamic_obstacles;
    RobotInstant startInstant = request.start;
    const RobotConstraints& constraints = request.constraints;

    const Geometry2d::Rect &fieldRect = Field_Dimensions::Current_Dimensions.FieldRect();
    MotionInstant futureBallInstant = ball.predict(contactTime);
    Point futureBallPoint = futureBallInstant.pos;
    double futureBallSpeed = futureBallInstant.vel.mag();
    bool returnPath = false;
    if (!fieldRect.containsPoint(futureBallPoint)) {
        futureBallPoint = projectPointIntoField(futureBallPoint, fieldRect,
                                                ball.pos);
        contactTime = ball.estimateTimeTo(futureBallPoint, &futureBallPoint);
        returnPath = true;
    }

    // collect candidate target states
    // we might try approaching the ball from a few different directions to see
    // which one is faster
    std::vector<double> contactSpeeds;
    std::vector<double> contactAngles;
    std::vector<Point> contactDirs;
    if (ball.vel.mag() > *_maxBallSpeedDirect) {
        if(ball.vel.mag() > *_maxOutrunBallSpeedPercent * constraints.mot.maxSpeed) {
            // the ball is moving fast toward us, so approach opposite ball.vel
            contactSpeeds.push_back(ball.vel.mag() * *_ballSpeedPercentForDampen);
            contactDirs.push_back(ball.vel.norm());
            contactAngles.push_back(ball.vel.angle() + M_PI);
        } else {
            // Try both Settle and Collect and pick the quicker one
            contactSpeeds.push_back(ball.vel.mag() * *_ballSpeedPercentForDampen);
            contactDirs.push_back(ball.vel.norm());
            contactAngles.push_back(ball.vel.angle() + M_PI);
            contactSpeeds.push_back(ball.vel.mag() + *_touchDeltaSpeed);
            contactDirs.push_back(ball.vel.norm());
            contactAngles.push_back(ball.vel.angle());
        }
    } else {
        if(command.targetSpeed) {
            contactSpeeds.push_back(*command.targetSpeed);
        } else {
            contactSpeeds.push_back(*_touchDeltaSpeed);
        }
        contactDirs.push_back((ball.pos - startInstant.pose.position()).norm());
        contactAngles.push_back(startInstant.pose.position().angleTo(ball.pos));
    }
    assert(contactDirs.size() == contactAngles.size() && contactAngles.size() == contactSpeeds.size());

    // Check each candidate target state and pick the trajectory that gets
    // the ball the fastest
    Trajectory outputPath{{}};
    RJ::Time outputPathContactBallTime = RJ::Time::max();
    constexpr double minMaxContactSpeed = 0.1;
    for(int i = 0; i < contactDirs.size(); i++) {
        Point faceDir;
        if(command.targetFacePoint && command.targetSpeed) {
            faceDir = (*command.targetFacePoint - futureBallPoint).norm();
        } else {
            faceDir = Point::direction(contactAngles[i]);
        }

        Point contactPoint = futureBallPoint - (Robot_MouthRadius + Ball_Radius) * faceDir;
        double contactSpeed = contactSpeeds[i];
        Point contactDir = contactDirs[i];
        double contactAngle = faceDir.angle();
        assert(contactSpeed <= constraints.mot.maxSpeed);

        if(Point::direction(contactAngle).angleBetween(faceDir) > *_maxApproachAngle) {
            continue;
        }

        // Course: approach the ball in line with the ball velocity
        std::optional<Trajectory> coursePath;
        Point courseTargetPoint = contactPoint - contactDir.normalized(*_bufferDistBeforeContact);
        Geometry2d::Pose courseTargetPose{courseTargetPoint, 0};
        Geometry2d::Twist courseTargetTwist{contactDir.normalized(contactSpeed), 0};
        RobotInstant courseTargetInstant{courseTargetPose, courseTargetTwist, RJ::now()};
        Point botToContact = contactPoint-startInstant.pose.position();
        // robot is in line with the ball and close to it
        bool inlineWithBall = std::abs(botToContact.cross(contactDir)) < Robot_Radius/2.0;
        bool closeToBall = botToContact.dot(contactDir) < *_bufferDistBeforeContact;
        RJ::Time contactBallTime = RJ::Time::max();
        //if we aren't in the fine segment
        if(!(inlineWithBall && closeToBall)) {
            coursePath = RRTTrajectory(startInstant, courseTargetInstant, constraints.mot, static_obstacles, dynamic_obstacles);
            if(coursePath->empty()) continue;
            courseTargetInstant.stamp = coursePath->last().stamp;
            contactBallTime = coursePath->last().stamp;
        }

        //Fine: constant velocity while contacting the ball
        RobotConstraints fineConstraints = constraints;
        fineConstraints.mot.maxAcceleration *= *_ballContactAccelPercent;
        fineConstraints.mot.maxSpeed = std::max(contactSpeed, minMaxContactSpeed);
        //Fine Path Before Contact
        Geometry2d::Pose contactPose{contactPoint, contactAngle};
        Geometry2d::Twist contactTwist{contactDir.normalized(contactSpeed), 0};
        RobotInstant contactInstant{contactPose, contactTwist, RJ::now()};
        std::optional<Trajectory> finePathBeforeContact;
        bool isBeforeContact = coursePath || botToContact.mag() > Robot_MouthRadius + Ball_Radius;
        if(isBeforeContact) {
            RobotInstant instantBeforeFine = coursePath ? courseTargetInstant : startInstant;
            finePathBeforeContact = RRTTrajectory(instantBeforeFine, contactInstant, fineConstraints.mot, static_obstacles, dynamic_obstacles);
            if(finePathBeforeContact->empty()) continue;
            contactInstant.stamp = finePathBeforeContact->last().stamp;
        }
        contactBallTime = contactInstant.stamp;
        //Fine Path After Contact
        std::optional<Trajectory> finePathAfterContact;
        if(isBeforeContact) {
            double stoppingDist = std::pow(contactSpeed, 2) / (2 * fineConstraints.mot.maxAcceleration);
            Point fineTargetPoint = contactPoint + contactDir.normalized(*_bufferDistAfterContact + stoppingDist);
            RobotInstant fineTargetInstant{Geometry2d::Pose{fineTargetPoint, 0}, {}, RJ::Time(0s)};
            finePathAfterContact = RRTTrajectory(contactInstant, fineTargetInstant, fineConstraints.mot, static_obstacles, dynamic_obstacles);
            if(finePathAfterContact->empty()) continue;
        }

        //combine all the trajectory segments to build the output path
        Trajectory collectPath{{}};
        if(coursePath) {
            assert(finePathBeforeContact && finePathAfterContact);
            Trajectory finePath{std::move(*finePathBeforeContact), std::move(*finePathAfterContact)};
            collectPath = Trajectory{std::move(*coursePath), std::move(finePath)};
            collectPath.setDebugText("Course");
//        } else if (finePathBeforeContact) {
//            assert(finePathAfterContact);
//            collectPath = Trajectory{std::move(*finePathBeforeContact), std::move(*finePathAfterContact)};
//            collectPath.setDebugText("Fine");
        } else {
            //we're too close to the ball to trust anything. just reuse old path
            return std::nullopt;
        }
        assert(!collectPath.empty());
        if(outputPath.empty() || contactBallTime < outputPathContactBallTime) {
            outputPath = std::move(collectPath);
            PlanAngles(outputPath, startInstant, AngleFns::tangent/*todo(Ethan)fixthisAngleFns::faceAngle(contactAngle)*/, constraints.rot);
            outputPathContactBallTime = contactBallTime;
        }
    }
    return std::move(outputPath);
}
}