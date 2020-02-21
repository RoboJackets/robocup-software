#include "CapturePlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "Geometry2d/Pose.hpp"
#include "Geometry2d/Rect.hpp"
#include <functional>
#include "motion/TrapezoidalMotion.hpp"

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

double CapturePlanner::ballSpeedPercentForDampen() {
    return *_ballSpeedPercentForDampen;
}
double CapturePlanner::touchDeltaSpeed() {
    return *_touchDeltaSpeed;
}

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
    Point currentFaceDir = Point::direction(request.start.pose.heading());
    const double ballDistFront = (-ballToBot).dot(currentFaceDir);
    const double ballDistSide = (-ballToBot).cross(currentFaceDir);
    Point ballToBotUnit = ballToBot.norm();
    // the non-negative component of ball.vel pointing toward the robot
    Point ballVelToBot = std::max(0.0, ball.vel.dot(ballToBotUnit)) * ballToBotUnit;
    // the component of ball.vel the robot will have to outrun
    Point awayBallVelocity = ball.vel - ballVelToBot;
    if(awayBallVelocity.mag() > *_maxOutrunBallSpeedPercent * constraints.mot.maxSpeed) {
        //Impossible Capture Request. The ball is running away too fast
        //Todo(Ethan) handle this probably
        return reuse(std::move(request));
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
        bool ballFarAway = ballDistSide > Robot_Radius || ballDistFront > 3 * Robot_MouthRadius;
        bool goalChanged = prevGoalPos.distTo(curGoalPos) > 0.05; //todo(Ethan) no magic numbers
        bool goalDistToBallLine = std::abs((prevGoalPos-ball.pos).cross(ball.vel.norm()));
        if(!partialReplan && (goalChanged && ballFarAway || ball.vel.mag() > 0.2 && ballToBot.mag() > Robot_Radius*1.5 && goalDistToBallLine > Robot_Radius)) {
            partialReplan = true;
            invalidTime = prevTrajectory.duration();//todo(Ethan) delete comment std::min(prevTrajectory.duration(), RJ::Seconds(2 * *_partialReplanLeadTime));
        }
        if(partialReplan && invalidTime < timeElapsed + RJ::Seconds(2 * *_partialReplanLeadTime)) {
            fullReplan = true;
        }
    }
//    if(mouthNormalDist > 0 && mouthNormalDist < Robot_MouthRadius && std::abs(mouthOffCenterDist) < )
    if(ballToBot.mag() < Robot_MouthRadius) {
        fullReplan = partialReplan = false;
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
    constexpr RJ::Seconds maxSearchDuration = 3s;
    constexpr RJ::Seconds brute_inc = 0.1s;
    std::optional<Trajectory> path;
    RJ::Time startTime = RJ::now();
    Point botPos = request.start.pose.position();
    double botSpeed = request.start.velocity.linear().mag();
    double distToBallLine = std::abs(ball.vel.norm().cross(ball.pos-botPos));
    double timeEstimate = Trapezoidal::getTime(distToBallLine, distToBallLine, request.constraints.mot.maxSpeed,request.constraints.mot.maxAcceleration, botSpeed, 0);
    RJ::Time contactTime = RJ::now() + RJ::Seconds{timeEstimate};
    while(contactTime < startTime + maxSearchDuration) {
        std::optional<Trajectory> candidatePath;
        bool successfulCapture = false;
        RJ::Time attemptt0 = RJ::now();
        auto pathResult = attemptCapture(request, contactTime);
        printf("   attempt took %.3f sec, its: %d\n", RJ::Seconds(RJ::now()-attemptt0).count(), its);//todo(Ethan) delete
        if(pathResult) {
            std::tie(candidatePath, successfulCapture) = *pathResult;
            if(successfulCapture) {
                // use the first successful path found
                path = candidatePath;
                break;
            } else if (candidatePath && !candidatePath->empty() && (!path || candidatePath->duration() < path->duration())) {
                // find the best path in case none of them are successful
                path = candidatePath;
                //todo(Ethan) delete this break
                break;
            }
        }
        contactTime = RJ::Time{contactTime + brute_inc};
        its++;
    }
    printf("brute force took %.3f sec, its: %d\n", RJ::Seconds(RJ::now()-startTime).count(), its);//todo(Ethan) delete
    if(path) {
        assert(!path->empty());
        return std::move(*path);
    }
    //sometimes RRT fails, so use the old path and try again next iteration
    return std::nullopt;
}

std::optional<std::tuple<Trajectory, bool>> CapturePlanner::attemptCapture(const PlanRequest& request, RJ::Time contactTime) const {
    const Ball& ball = request.context->state.ball;
    const CaptureCommand& command = std::get<CaptureCommand>(request.motionCommand);
    if(!(command.targetSpeed && command.targetFacePoint)) {
        debugThrow("Error: incomplete capture command");
    }
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

    Point faceDir;
    if(command.targetFacePoint->distTo(futureBallPoint) < 1e-9) {
        faceDir = (futureBallPoint - startInstant.pose.position()).norm();
    } else {
        faceDir = (*command.targetFacePoint - futureBallPoint).norm();
    }
    Point contactPoint = futureBallPoint - (Robot_MouthRadius + Ball_Radius) * faceDir;
    double contactSpeed = *command.targetSpeed;
    //todo(change contactDir during linekick for a one-touch?
    Point contactDir = faceDir;
    double contactAngle = faceDir.angle();
    assert(std::abs(contactSpeed) <= constraints.mot.maxSpeed);

    if(contactDir.angleBetween(faceDir) > *_maxApproachAngle) {
        debugThrow("Attack Angle in Capture too big");
        return std::nullopt;
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
    bool closeToBall = std::abs(botToContact.dot(contactDir)) < *_bufferDistBeforeContact;
    RJ::Time contactBallTime = RJ::Time::max();
    //if we aren't in the fine segment
    if(!(inlineWithBall && closeToBall)) {
        coursePath = RRTTrajectory(startInstant, courseTargetInstant, constraints.mot, static_obstacles, dynamic_obstacles);
        if(coursePath->empty()) {
            return std::nullopt;
        }
        courseTargetInstant.stamp = coursePath->last().stamp;
        contactBallTime = coursePath->last().stamp;
    } else {
        //todo(Ethan) Really? (see below todo)
        return std::nullopt;
    }

    //Fine: constant velocity while contacting the ball
    RobotConstraints fineConstraints = constraints;
    fineConstraints.mot.maxAcceleration *= *_ballContactAccelPercent;
    fineConstraints.mot.maxSpeed = std::abs(contactSpeed);
    std::optional<Trajectory> finePathBeforeContact, finePathAfterContact;
    // if we plan to stop before making contact with the ball (e.g. a Settle)
    // then we only do a course approach, and wait for the ball to roll into us
    if(fineConstraints.mot.maxSpeed > 1e-6) {
        //Fine Path Before Contact
        Geometry2d::Pose contactPose{contactPoint, contactAngle};
        Geometry2d::Twist contactTwist{contactDir.normalized(contactSpeed), 0};
        RobotInstant contactInstant{contactPose, contactTwist, RJ::now()};
        bool isBeforeContact = coursePath || botToContact.mag() > Robot_MouthRadius + Ball_Radius;
        if(isBeforeContact) {
            RobotInstant instantBeforeFine = coursePath ? courseTargetInstant : startInstant;
            finePathBeforeContact = RRTTrajectory(instantBeforeFine, contactInstant, fineConstraints.mot, static_obstacles, dynamic_obstacles);
            if(finePathBeforeContact->empty()) {
                if(coursePath) {
                    return std::make_tuple(std::move(*coursePath), false);
                }
                return std::nullopt;
            }
            contactInstant.stamp = finePathBeforeContact->last().stamp;
        }

        contactBallTime = contactInstant.stamp;
        //Fine Path After Contact
        if(isBeforeContact) {
            double stoppingDist = std::pow(contactSpeed, 2) / (2 * fineConstraints.mot.maxAcceleration);
            Point fineTargetPoint = contactPoint + contactDir.normalized(*_bufferDistAfterContact + stoppingDist);
            RobotInstant fineTargetInstant{Geometry2d::Pose{fineTargetPoint, 0}, {}, RJ::Time(0s)};
            finePathAfterContact = RRTTrajectory(contactInstant, fineTargetInstant, fineConstraints.mot, static_obstacles, dynamic_obstacles);
            if(finePathAfterContact->empty()) {
                if(coursePath && finePathBeforeContact) {
                    return std::make_tuple(Trajectory{std::move(*coursePath), std::move(*finePathBeforeContact)}, false);
                } else if(coursePath) {
                    return std::make_tuple(std::move(*coursePath), false);
                }
                return std::nullopt;
            }
        }
    } else {
        if(coursePath) {
            return std::make_tuple(std::move(*coursePath), false);
        } else {
            return std::nullopt;
        }
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
        //todo(Ethan) REeally?
        return std::nullopt;
    }
    assert(!collectPath.empty());
    bool successfulCapture = contactBallTime - 1e-6s < contactTime;
    PlanAngles(collectPath, startInstant, AngleFns::faceAngle(contactAngle), constraints.rot);
    return std::make_tuple(std::move(collectPath), successfulCapture);
}
}