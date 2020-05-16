#include "SettlePlanner.hpp"
#include "motion/TrapezoidalMotion.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"

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
    _bufferTimeBeforeContact =
        new ConfigDouble(cfg, "Capture/Settle/bufferTimeBeforeContact");
    _targetPointGain = new ConfigDouble(cfg, "Capture/Settle/targetPointGain");
    _searchStartDist = new ConfigDouble(cfg, "Capture/Settle/searchStartDist");
    _searchEndDist = new ConfigDouble(cfg, "Capture/Settle/searchEndDist");
    _searchIncDist = new ConfigDouble(cfg, "Capture/Settle/searchIncDist");
    _targetChangeThreshold =
        new ConfigDouble(cfg, "Capture/Settle/targetChangeThreshold");
    _shortcutDist = new ConfigDouble(cfg, "Capture/Settle/shortcutDist");
}
using namespace Geometry2d;
// todo(motion planning) find a better way to handle moving targets
RJ::Time SettlePlanner::bruteForceIntercept(const PlanRequest& request) {
    BallState ball = request.world_state->ball;
    std::vector<double> dists;
    double endDist = *_searchEndDist;
    const Rect& fieldRect = Field_Dimensions::Current_Dimensions.FieldRect();
    Segment ballLine{ball.position, ball.position + ball.velocity.normalized(endDist)};
    // stop the search early if we go out of bounds
    auto [isBoundaryHit, boundaryHits] = fieldRect.intersects(ballLine);
    if (isBoundaryHit && boundaryHits.size() > 0) {
        Point boundPoint = boundaryHits[0];
        endDist = std::min(endDist, ball.position.distTo(boundPoint));
    }
    for (double x = *_searchStartDist; x < endDist; x += *_searchIncDist) {
        dists.push_back(x);
    }
    // brute force to find the path with the soonest contact time
    RJ::Time interceptTime = RJ::Time::max();
    _pathTargetPlanner.drawRadius = Ball_Radius;
    _pathTargetPlanner.drawColor = Qt::white;
    _pathTargetPlanner.drawLayer = "Settle Brute Force";
//#pragma omp parallel for default(none) \
    shared(request, dists, ball, interceptTime)
    for (int i = 0; i < dists.size(); i++) {
        std::optional<RJ::Seconds> maybe_contact_duration =
            ball.query_seconds_to_dist(dists[i]);

        // If the ball will never reach this distance, we have no use for it.
        if (!maybe_contact_duration) {
            continue;
        }

        RJ::Time contact_time = ball.timestamp + maybe_contact_duration.value();

        Trajectory path = intercept(request.copyNoHistory(), contact_time);
        if (!path.empty()) {
//#pragma omp critical
            {
                if (path.end_time() + RJ::Seconds{*_bufferTimeBeforeContact} <
                        contact_time &&
                    contact_time < interceptTime) {
                    interceptTime = contact_time;
                }
            }
        }
    }
    return interceptTime;
}

Trajectory SettlePlanner::intercept(PlanRequest&& request,
                                    RJ::Time interceptTime) {
    BallState ball = request.world_state->ball;
    Point ballVel = ball.velocity;
    RobotInstant startInstant = request.start;
    RotationConstraints rotationConstraints = request.constraints.rot;
    Point ballPointAtContact = ball.predict_at(interceptTime).position;
    Point robotPointAtContact =
        ballPointAtContact +
        ballVel.normalized(Ball_Radius + Robot_MouthRadius);
    request.static_obstacles.add(std::make_shared<Circle>(
        ballPointAtContact - ball.velocity.norm() * .1,
        Ball_Radius));  // subtract a bit from ballPointAtContact to prevent a
                        // collision detection
    RobotInstant goalInstant{Pose{robotPointAtContact, 0}, {}, RJ::Time{0s}};
    request.motionCommand = PathTargetCommand{goalInstant};
    Trajectory path = _pathTargetPlanner.plan(std::move(request));
    PlanAngles(path, startInstant, AngleFns::faceAngle(ballVel.angle() + M_PI),
               rotationConstraints);
    return std::move(path);
}
Trajectory SettlePlanner::plan(PlanRequest&& request) {
    RobotInstant startInstant = request.start;
    RotationConstraints rotationConstraints = request.constraints.rot;
    BallState ball = request.world_state->ball;
    Point ballVel = ball.velocity;
    // search for a goal with brute force
    Pose targetPose = findTargetPose(request);
    Segment ballLine{ball.position, targetPose.position()};

    if (request.debug_drawer != nullptr) {
        request.debug_drawer->drawLine(ballLine, Qt::white, "Planning");
        request.debug_drawer->drawCircle(
            targetPose.position(), Robot_MouthRadius, Qt::black, "Planning");
        request.debug_drawer->drawLine(
            targetPose.position(),
            targetPose.position() +
                Point::direction(targetPose.heading()) * 2 * Robot_Radius,
            Qt::black, "Planning");
    }

    // Shortcut Path: if we are almost done, move directly toward the ball line
    // This prevents the planner from suddenly changing the target just before
    // we get there because it thinks we won't make it in time.
    Point closestPt = ballLine.nearestPoint(startInstant.pose.position());
    bool inFrontOfBall = ball.velocity.dot(closestPt - ball.position) > 0;
    bool withinShortcut =
        startInstant.pose.position().distTo(closestPt) <
        *_shortcutDist;  // && closestPt.distTo(targetPose.position()) <
                         // *_shortcutDist;
    if (inFrontOfBall && withinShortcut) {
        RobotInstant goalInstant{Pose{closestPt, 0}, {}, RJ::Time{0s}};
        Trajectory path = CreatePath::simple(startInstant, goalInstant,
                                             request.constraints.mot);
        if (!path.empty()) {
            PlanAngles(path, startInstant,
                       AngleFns::faceAngle(ball.velocity.angle() + M_PI),
                       rotationConstraints);
            return std::move(path);
        }
    }

    // plan toward the goal with PathTargetPlanner
    RobotInstant targetInstant{targetPose, {}, RJ::Time{0s}};
    request.motionCommand = PathTargetCommand{targetInstant};
    // add ball obstacle at contact time and current time
    request.static_obstacles.add(std::make_shared<Circle>(
        targetPose.position() +
            Point::direction(targetPose.position().angle()) *
                (Robot_MouthRadius + Ball_Radius + .1),
        Ball_Radius));
    request.static_obstacles.add(
        std::make_shared<Circle>(ball.position, Ball_Radius));
    Trajectory path = _pathTargetPlanner.plan(std::move(request));
    PlanAngles(path, startInstant, AngleFns::faceAngle(ballVel.angle() + M_PI),
               rotationConstraints);
    return std::move(path);
}
Pose SettlePlanner::findTargetPose(const PlanRequest& request) {
    RJ::Time interceptTime = bruteForceIntercept(request);
    BallState ball = request.world_state->ball;
    Point noisyTargetPoint =
        ball.predict_at(interceptTime).position +
        ball.velocity.normalized(Ball_Radius + Robot_MouthRadius);

    // if the intercept target is out of bounds, project it into the field
    const Rect& fieldRect = Field_Dimensions::Current_Dimensions.FieldRect();
    Segment ballLine{ball.position, noisyTargetPoint};
    auto [isBoundaryHit, boundaryHits] = fieldRect.intersects(ballLine);
    if (isBoundaryHit && boundaryHits.size() > 0) {
        noisyTargetPoint = boundaryHits[0];
    }

    std::optional<Point>& avgTargetPoint = _avgTargetPoints[request.shellID];
    // reduce high frequency noise by applying a low pass filter
    if (!avgTargetPoint) {
        avgTargetPoint = noisyTargetPoint;
    } else {
        avgTargetPoint = applyLowPassFilter(*avgTargetPoint, noisyTargetPoint,
                                            *_targetPointGain);
    }
    // reduce low frequency movement by applying a threshold on position change
    std::optional<Point>& targetPoint = _targetPoints[request.shellID];
    if (!targetPoint ||
        targetPoint->distTo(*avgTargetPoint) > *_targetChangeThreshold) {
        targetPoint = *avgTargetPoint;
    }
    return Pose{*targetPoint, ball.velocity.angle() + M_PI};
}
}  // namespace Planning