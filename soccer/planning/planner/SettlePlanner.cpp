#include "SettlePlanner.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"
#include "motion/TrapezoidalMotion.hpp"

namespace Planning {
ConfigDouble* SettlePlanner::_bufferTimeBeforeContact;
ConfigDouble* SettlePlanner::_targetPointGain;
ConfigDouble* SettlePlanner::_searchStartDist;
ConfigDouble* SettlePlanner::_searchEndDist;
ConfigDouble* SettlePlanner::_searchIncDist;
ConfigDouble* SettlePlanner::_targetChangeThreshold;
ConfigDouble* SettlePlanner::_shortcutDist;

REGISTER_CONFIGURABLE(SettlePlanner);

void SettlePlanner::createConfiguration(Configuration* cfg) {
    _bufferTimeBeforeContact = new ConfigDouble(cfg,
                                                "Capture/Settle/bufferTimeBeforeContact");
    _targetPointGain = new ConfigDouble(cfg,
                                          "Capture/Settle/targetPointGain");
    _searchStartDist = new ConfigDouble(cfg,
                                        "Capture/Settle/searchStartDist");
    _searchEndDist = new ConfigDouble(cfg,
                                      "Capture/Settle/searchEndDist");
    _searchIncDist = new ConfigDouble(cfg,
                                      "Capture/Settle/searchIncDist");
    _targetChangeThreshold = new ConfigDouble(cfg,
                                              "Capture/Settle/targetChangeThreshold");
    _shortcutDist = new ConfigDouble(cfg,
                                     "Capture/Settle/shortcutDist");
}
using namespace Geometry2d;
//todo(motion planning) find a better way to handle moving targets
RJ::Time SettlePlanner::bruteForceIntercept(const PlanRequest& request) {
    Ball& ball = request.context->state.ball;
    std::vector<double> dists;
    double endDist = *_searchEndDist;
    const Rect &fieldRect = Field_Dimensions::Current_Dimensions.FieldRect();
    Segment ballLine{ball.pos, ball.pos + ball.vel.normalized(endDist)};
    // stop the search early if we go out of bounds
    auto [isBoundaryHit, boundaryHits] = fieldRect.intersects(ballLine);
    if (isBoundaryHit && boundaryHits.size() > 0) {
        Point boundPoint = boundaryHits[0];
        endDist = std::min(endDist, ball.pos.distTo(boundPoint));
    }
    for(double x = *_searchStartDist; x < endDist; x += *_searchIncDist) {
        dists.push_back(x);
    }
    // Run brute force using a parallel for loop thanks to OpenMP
    // considering the interceptBufferTime constraint:
    // if the constraint can be satisfied, find the path of minimum contact time
    // if not, find the path with maximum intercept buffer time
    // todo(Ethan) ^^^ actually implement that...
    RJ::Time interceptTime = ball.estimateTimeTo(ball.pos + ball.vel.normalized(endDist));
    _pathTargetPlanner.drawRadius = Ball_Radius;
    _pathTargetPlanner.drawColor = Qt::white;
    _pathTargetPlanner.drawLayer = "Settle Brute Force";
#pragma omp parallel for default(none) shared(request, dists, ball, interceptTime)
    for(int i = 0; i < dists.size(); i++) {
        RJ::Time contactTime = ball.estimateTimeTo(ball.pos + ball.vel.normalized(dists[i]));
        //todo(Ethan) full copy or request.copyNoHistory()?
        Trajectory path = intercept(request.copyNoHistory(), contactTime);
        if(!path.empty()) {
#pragma omp critical
{
            if (path.end_time() + RJ::Seconds{*_bufferTimeBeforeContact} < contactTime
            && contactTime < interceptTime) {
                interceptTime = contactTime;
            }
}
        }
    }
    return interceptTime;
}
Trajectory SettlePlanner::intercept(PlanRequest&& request, RJ::Time interceptTime) {
    const Ball& ball = request.context->state.ball;
    Point ballVel = ball.vel;
    RobotInstant startInstant = request.start;
    RotationConstraints rotationConstraints = request.constraints.rot;
    Point ballPointAtContact = ball.predict(interceptTime).pos;
    Point robotPointAtContact = ballPointAtContact + ballVel.normalized(
            Ball_Radius + Robot_MouthRadius);
    request.static_obstacles.add(std::make_shared<Circle>(ballPointAtContact - ball.vel.norm() * .1, Ball_Radius)); // subtract a bit from ballPointAtContact to prevent a collision detection
    RobotInstant goalInstant{Pose{robotPointAtContact, 0}, {}, RJ::Time{0s}};
    request.motionCommand = PathTargetCommand{goalInstant};
    Trajectory path = _pathTargetPlanner.plan(std::move(request));
    PlanAngles(path, startInstant, AngleFns::faceAngle(ballVel.angle()+M_PI),
            rotationConstraints);
    return std::move(path);
}
Trajectory SettlePlanner::plan(PlanRequest&& request) {
    RobotInstant startInstant = request.start;
    RotationConstraints rotationConstraints = request.constraints.rot;
    const Ball& ball = request.context->state.ball;
    Point ballVel = ball.vel;
    Pose targetPose = findTargetPose(request);
    Segment ballLine{ball.pos, targetPose.position()};
    request.context->debug_drawer.drawLine(ballLine, Qt::white, "Planning");
    request.context->debug_drawer.drawCircle(targetPose.position(), Robot_MouthRadius, Qt::black, "Planning");
    request.context->debug_drawer.drawLine(targetPose.position(), targetPose.position() + Point::direction(targetPose.heading()) * 2 * Robot_Radius, Qt::black, "Planning");

    // Shortcut Path: if we are almost done, move directly toward the ball line
    // This prevents the planner from suddenly changing the target just before
    // we get there because it thinks we won't make it in time.
    Point closestPt = ballLine.nearestPoint(startInstant.pose.position());
    bool inFrontOfBall = ball.vel.dot(closestPt - ball.pos) > 0;
    auto& avgTarget = _avgTargetBallPoints[request.shellID];
    bool withinShortcut = startInstant.pose.position().distTo(closestPt) < *_shortcutDist;// && avgTarget && closestPt.distTo(targetPose.position()) < *_shortcutDist;
    if(inFrontOfBall && withinShortcut) {
        RobotInstant goalInstant{Pose{closestPt,0}, {}, RJ::Time{0s}};
        Trajectory path = CreatePath::simple(startInstant, goalInstant, request.constraints.mot);
        if (!path.empty()) {
            printf("shortcut\n");
            PlanAngles(path, startInstant, AngleFns::faceAngle(ball.vel.angle()+M_PI),rotationConstraints);
            return std::move(path);
        } else {
            printf("failed shortcut\n");
        }
    }

    // Brute Force Path: search for a goal with brute force, then plan toward
    // the goal with PathTargetPlanner
    RobotInstant targetInstant{targetPose, {}, RJ::Time{0s}};
    request.motionCommand = PathTargetCommand{targetInstant};
    // add ball obstacle at contact time and current time
    request.static_obstacles.add(std::make_shared<Circle>(targetPose.position() + Point::direction(targetPose.position().angle()) * (Robot_MouthRadius+Ball_Radius+.1), Ball_Radius));
    request.static_obstacles.add(std::make_shared<Circle>(ball.pos, Ball_Radius));
    Trajectory path = _pathTargetPlanner.plan(std::move(request));
    PlanAngles(path, startInstant, AngleFns::faceAngle(ballVel.angle()+M_PI),
               rotationConstraints);
    return std::move(path);
}
Pose SettlePlanner::findTargetPose(const PlanRequest& request) {
    RJ::Time interceptTime = bruteForceIntercept(request);
    const Ball& ball = request.context->state.ball;
    Point noisyTargetBallPoint = ball.predict(interceptTime).pos;

    // if the intercept target is out of bounds, project it into the field
    const Rect &fieldRect = Field_Dimensions::Current_Dimensions.FieldRect();
    Segment ballLine{ball.pos, noisyTargetBallPoint};
    auto [isBoundaryHit, boundaryHits] = fieldRect.intersects(ballLine);
    if (isBoundaryHit && boundaryHits.size() > 0) {
        noisyTargetBallPoint = boundaryHits[0];
    }

    std::optional<Point>& avgTargetBallPoint = _avgTargetBallPoints[request.shellID];
    // reduce high frequency noise by applying a low pass filter
    if(!avgTargetBallPoint) {
        avgTargetBallPoint = noisyTargetBallPoint;
    } else {
        avgTargetBallPoint = applyLowPassFilter(*avgTargetBallPoint, noisyTargetBallPoint, *_targetPointGain);
    }
    // reduce low frequency movement by applying a threshold on position change
    std::optional<Point>& targetBallPoint = _targetBallPoints[request.shellID];
    if (!targetBallPoint || targetBallPoint->distTo(*avgTargetBallPoint) > *_targetChangeThreshold) {
        targetBallPoint = *avgTargetBallPoint;
    }
    return Pose{*targetBallPoint + ball.vel.normalized(Ball_Radius+Robot_MouthRadius), ball.vel.angle() + M_PI};
}
}