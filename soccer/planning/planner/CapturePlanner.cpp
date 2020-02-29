#include "CapturePlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "Geometry2d/Pose.hpp"
#include "Geometry2d/Rect.hpp"
#include <functional>
#include "motion/TrapezoidalMotion.hpp"

namespace Planning {
using namespace Geometry2d;

REGISTER_CONFIGURABLE(CapturePlanner);

ConfigDouble* CapturePlanner::_searchStartDist;
ConfigDouble* CapturePlanner::_searchEndDist;
ConfigDouble* CapturePlanner::_searchIncDist;
ConfigDouble* CapturePlanner::_maxBallSpeedDirect;
ConfigDouble* CapturePlanner::_maxApproachAngle;
ConfigDouble* CapturePlanner::_maxOutrunBallSpeedPercent;
ConfigDouble* CapturePlanner::_ballContactAccelPercent;
ConfigDouble* CapturePlanner::_collectBufferDistBeforeContact;
ConfigDouble* CapturePlanner::_collectBufferDistAfterContact;
ConfigDouble* CapturePlanner::_settleBufferTimeBeforeContact;
ConfigDouble* CapturePlanner::_touchDeltaSpeed;
ConfigDouble* CapturePlanner::_ballSpeedPercentForDampen;
ConfigDouble* CapturePlanner::_collectBallSpeedApproachDirectionCutoff;
ConfigDouble* CapturePlanner::_settleTargetSensitivity;

void CapturePlanner::createConfiguration(Configuration* cfg) {
    _searchStartDist = new ConfigDouble(cfg,
        "Capture/searchStartDist",0);
    _searchEndDist = new ConfigDouble(cfg,
        "Capture/searchEndDist",0);
    _searchIncDist = new ConfigDouble(cfg,
        "Capture/searchIncDist",0);
    _maxBallSpeedDirect = new ConfigDouble(cfg,
        "Capture/maxBallSpeedDirect", 0);
    _maxOutrunBallSpeedPercent = new ConfigDouble(cfg,
        "Capture/maxOutrunBallSpeed", 0);
    _maxApproachAngle = new ConfigDouble(cfg,
        "Capture/maxApproachAngle", 0);
    _ballContactAccelPercent = new ConfigDouble(cfg,
        "Capture/ballContactAccelPercent", 0);
    _collectBufferDistBeforeContact = new ConfigDouble(cfg,
                                                       "Capture/Collect/bufferDistBeforeContact", 0);
    _collectBufferDistAfterContact = new ConfigDouble(cfg,
                                                      "Capture/Collect/bufferDistAfterContact", 0);
    _settleBufferTimeBeforeContact = new ConfigDouble(cfg,
            "Capture/Settle/bufferTimeBeforeContact", 0);
    _touchDeltaSpeed = new ConfigDouble(cfg,
        "Capture/touchDeltaSpeed", 0);
    _ballSpeedPercentForDampen = new ConfigDouble(cfg,
            "Capture/ballSpeedPercentForDampen", 0);
    _collectBallSpeedApproachDirectionCutoff = new ConfigDouble(cfg,
            "Capture/Collect/ballSpeedApproachDirectionCutoff",0);
    _settleTargetSensitivity = new ConfigDouble(cfg,
            "Capture/Settle/targetSensitivity", 0);
}
RobotInstant CapturePlanner::getGoalInstant(const PlanRequest& request) {
    auto bruteForceResult = bruteForceCapture(request);
    Trajectory path{{}};
    if(bruteForceResult) {
        std::tie(_contactTime, path) = std::move(*bruteForceResult);
    }
    if(path.empty()) {
        Point ballPoint = request.context->state.ball.pos;
        Point startPoint = request.start.pose.position();
        return RobotInstant{Pose{ballPoint,startPoint.angleTo(ballPoint)}, {}, RJ::now()};
    }
    double a = *_settleTargetSensitivity;
    Point targetPoint = path.last().pose.position();
    if(!avgTargetPoint) {
        avgTargetPoint = targetPoint;
    } else {
        avgTargetPoint = a * targetPoint + (1-a) * *avgTargetPoint;
    }
    RobotInstant targetInstant = path.last();
    targetInstant.pose.position() = *avgTargetPoint;
    return targetInstant;

}

Trajectory CapturePlanner::checkBetter(PlanRequest&& request, RobotInstant goalInstant, AngleFunction angleFunction) {
    _contactTime = std::max(_contactTime, RJ::now());
    Trajectory prevTrajectoryCopy = request.prevTrajectory;
    Trajectory newTrajectory = partialReplan(std::move(request), goalInstant, angleFunction);
    if(newTrajectory.end_time() < prevTrajectoryCopy.end_time()) {
        return std::move(newTrajectory);
    }
    return std::move(prevTrajectoryCopy);
}
Trajectory CapturePlanner::partialReplan(PlanRequest&& request, RobotInstant goalInstant,   AngleFunction angleFunction) {
    _contactTime = std::max(_contactTime, RJ::now());
    RobotInstant startInstant = request.start;
    Trajectory partial_pre = partialPath(request.prevTrajectory);
    request.start = partial_pre.last();
    std::optional<std::tuple<Trajectory, bool>> captureResult = attemptCapture(request, _contactTime);
    if(!captureResult) {
        return reuse(std::move(request));
    }
    Trajectory partial_post = std::move(std::get<0>(*captureResult));
    return Trajectory{std::move(partial_pre), std::move(partial_post)};
}
Trajectory CapturePlanner::fullReplan(PlanRequest&& request, RobotInstant goalInstant, AngleFunction angleFunction) {
    _contactTime = std::max(_contactTime, RJ::now());
    auto captureResult = attemptCapture(request, _contactTime);
    if(!captureResult) {
        return reuse(std::move(request));
    }
    return std::move(std::get<0>(*captureResult));
}
//todo(motion planning) find a better way to handle moving targets
std::optional<std::tuple<RJ::Time, Trajectory>> CapturePlanner::bruteForceCapture(const PlanRequest& request) const {
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
        return std::make_tuple(contactTime, std::move(*path));
    }
    //sometimes RRT fails, so use the old path and try again next iteration
    return std::nullopt;
}

std::tuple<Point, Point, RJ::Time, bool> CapturePlanner::predictFutureBallState(const Ball& ball, RJ::Time contactTime) const {
    // Consider the current state of the ball, predict the future state of the ball
    const Rect &fieldRect = Field_Dimensions::Current_Dimensions.FieldRect();
    MotionInstant futureBallInstant = ball.predict(contactTime);
    Point futureBallPoint = futureBallInstant.pos;
    Point futureBallVel = futureBallInstant.vel;
    bool fake = false;
    if (!fieldRect.containsPoint(futureBallPoint)) {
        fake = true;
        futureBallPoint = projectPointIntoField(futureBallPoint, fieldRect,
                                                ball.pos);
        contactTime = ball.estimateTimeTo(futureBallPoint, &futureBallPoint);
    }
    return std::make_tuple(futureBallPoint, futureBallVel, contactTime, fake);
}

Point CapturePlanner::projectPointIntoField(Point targetPoint, const Rect& fieldRect, Point ballPoint) const {
    auto intersectReturn = fieldRect.intersects(Segment(ballPoint, targetPoint));

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
//todo(Ethan) add ball as a dyn obs
std::optional<std::tuple<Trajectory, bool>> CapturePlanner::attemptCapture(const PlanRequest& request, RJ::Time contactTime) const {
    const Ball& ball = request.context->state.ball;
    const ShapeSet& static_obstacles = request.static_obstacles;
    const std::vector<DynamicObstacle>& dynamic_obstacles = request.dynamic_obstacles;
    RobotInstant startInstant = request.start;
    const RobotConstraints& constraints = request.constraints;

    Point futureBallPoint, futureBallVel;
    bool fakeFutureBall = false;
    std::tie(futureBallPoint, futureBallVel,contactTime,
            fakeFutureBall) = predictFutureBallState(ball, contactTime);

    ShapeSet static_obstacles_with_ball = static_obstacles;

        // Calculate the desired state of the robot at the point of ball contact
    // based on the command type
    Point targetFacePoint;
    double contactSpeed = 0;
    double bufferDistBeforeContact = 0;
    if(std::holds_alternative<CollectCommand>(request.motionCommand)) {
        if(futureBallVel.mag() < *_collectBallSpeedApproachDirectionCutoff) {
            targetFacePoint = ball.pos + (ball.pos - startInstant.pose.position()).normalized(10);
        } else {
            targetFacePoint = ball.pos + futureBallVel.normalized(10);
        }
        contactSpeed = futureBallVel.mag() + *_touchDeltaSpeed;
        bufferDistBeforeContact = *_collectBufferDistBeforeContact;
        static_obstacles_with_ball.add(std::make_shared<Circle>(futureBallPoint, Robot_Radius + Ball_Radius));
    } else if (std::holds_alternative<SettleCommand>(request.motionCommand)) {
        if(futureBallVel.mag() < *_collectBallSpeedApproachDirectionCutoff) {
            targetFacePoint = ball.pos + (ball.pos - startInstant.pose.position()).normalized(10);
        } else {
            targetFacePoint = ball.pos - futureBallVel.normalized(10);
        }
        contactSpeed = -futureBallVel.mag() * *_ballSpeedPercentForDampen;
        double timeUntilContact = RJ::Seconds{contactTime-RJ::now()}.count();
        if(!fakeFutureBall && *_settleBufferTimeBeforeContact > 0 &&
         timeUntilContact > *_settleBufferTimeBeforeContact ) {
            bool bufferFake = false;
            Point bufferBallPt, bufferBallVel;
            RJ::Time bufferTime = contactTime + RJ::Seconds(*_settleBufferTimeBeforeContact);
            std::tie(bufferBallPt, bufferBallVel, bufferTime, bufferFake) = predictFutureBallState(ball, bufferTime);
            if(!bufferFake) {
                bufferDistBeforeContact = (bufferBallPt - futureBallPoint).mag();
            }
        }
        static_obstacles_with_ball.add(std::make_shared<Circle>(ball.pos, Robot_Radius + Ball_Radius));
    } else if (std::holds_alternative<LineKickCommand>(request.motionCommand)) {
        auto lineKickCommand = std::get<LineKickCommand>(request.motionCommand);
        targetFacePoint = lineKickCommand.target;
        contactSpeed = lineKickApproachSpeed;
        bufferDistBeforeContact = *_collectBufferDistBeforeContact;
    } else {
        debugThrow("Invalid Command Type for Capture");
    }

    Point contactFaceDir;
    if(targetFacePoint.distTo(futureBallPoint) < 1e-9) {
        contactFaceDir = (futureBallPoint - startInstant.pose.position()).norm();
    } else {
        contactFaceDir = (targetFacePoint - futureBallPoint).norm();
    }
    //todo(change contactDir during linekick for a one-touch?
    Point contactDir = contactFaceDir;

    Point contactPoint = futureBallPoint - (Robot_MouthRadius + Ball_Radius) * contactFaceDir;
    contactSpeed = std::clamp(contactSpeed, -constraints.mot.maxSpeed, constraints.mot.maxSpeed);

    if(contactDir.angleBetween(contactFaceDir) > *_maxApproachAngle) {
        debugThrow("Attack Angle in Capture too big");
        return std::nullopt;
    }

    // Course: approach the ball in line with the ball velocity
    std::optional<Trajectory> coursePath;
    Point courseTargetPoint = contactPoint - contactDir.normalized(bufferDistBeforeContact);
    Pose courseTargetPose{courseTargetPoint, 0};
    Twist courseTargetTwist{contactDir.normalized(contactSpeed), 0};
    RobotInstant courseTargetInstant{courseTargetPose, courseTargetTwist, RJ::now()};
    Point botToContact = contactPoint-startInstant.pose.position();
    // robot is in line with the ball and close to it
    bool inlineWithBall = std::abs(botToContact.cross(contactDir)) < Robot_Radius/2.0;
    bool closeToBall = std::abs(botToContact.mag()) < *_collectBufferDistBeforeContact;
    RJ::Time contactBallTime = RJ::Time::max();
    //if we aren't in the fine segment
    if(ball.pos.distTo(startInstant.pose.position()) > bufferDistBeforeContact + Robot_Radius + Ball_Radius) {
        coursePath = RRTTrajectory(startInstant, courseTargetInstant, constraints.mot, static_obstacles_with_ball, dynamic_obstacles);
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
        Pose contactPose{contactPoint, contactFaceDir.angle()};
        Twist contactTwist{contactDir.normalized(contactSpeed), 0};
        RobotInstant contactInstant{contactPose, contactTwist, RJ::now()};
        bool isBeforeContact = coursePath || botToContact.mag() > Robot_MouthRadius + Ball_Radius;
        if(isBeforeContact) {
            RobotInstant instantBeforeFine = coursePath ? courseTargetInstant : startInstant;
            finePathBeforeContact = RRTTrajectory(instantBeforeFine, contactInstant, fineConstraints.mot, static_obstacles, dynamic_obstacles);
            if(finePathBeforeContact->empty()) {
                if(coursePath) {
                    Trajectory out = std::move(*coursePath);
                    PlanAngles(out, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
                    return std::make_tuple(std::move(out), false);
                }
                return std::nullopt;
            }
            contactInstant.stamp = finePathBeforeContact->last().stamp;
        }

        contactBallTime = contactInstant.stamp;
        //Fine Path After Contact
        if(isBeforeContact) {
            double stoppingDist = std::pow(contactSpeed, 2) / (2 * fineConstraints.mot.maxAcceleration);
            Point fineTargetPoint = contactPoint + contactDir.normalized(*_collectBufferDistAfterContact + stoppingDist);
            RobotInstant fineTargetInstant{Pose{fineTargetPoint, 0}, {}, RJ::Time(0s)};
            finePathAfterContact = RRTTrajectory(contactInstant, fineTargetInstant, fineConstraints.mot, static_obstacles, dynamic_obstacles);
            if(finePathAfterContact->empty()) {
                if(coursePath && finePathBeforeContact) {
                    Trajectory out{std::move(*coursePath), std::move(*finePathBeforeContact)};
                    PlanAngles(out, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
                    return std::make_tuple(std::move(out), false);
                } else if(coursePath) {
                    Trajectory out = std::move(*coursePath);
                    PlanAngles(out, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
                    return std::make_tuple(std::move(out), false);
                }
                return std::nullopt;
            }
        }
    } else {
        if(coursePath) {
            Trajectory out = std::move(*coursePath);
            PlanAngles(out, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
            return std::make_tuple(std::move(out), false);
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
    PlanAngles(collectPath, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
    return std::make_tuple(std::move(collectPath), successfulCapture);
}
}