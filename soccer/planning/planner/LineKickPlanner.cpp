#include "LineKickPlanner.hpp"

#include "planning/Instant.hpp"
#include "planning/primitives/RRTUtil.hpp"
namespace Planning {
ConfigDouble* LineKickPlanner::_approachSpeed;

REGISTER_CONFIGURABLE(LineKickPlanner);

void LineKickPlanner::createConfiguration(Configuration* cfg) {
    // NOLINTNEXTLINE
    _approachSpeed =
        new ConfigDouble(cfg, "PathPlanner/LineKickPlanner/approachSpeed");
}

using namespace Geometry2d;
Trajectory LineKickPlanner::plan(const PlanRequest& /* request */) {
#if 0
    Point aimTarget = std::get<LineKickCommand>(request.motionCommand).target;
    RobotInstant startInstant = request.start;
    BallState ball = request.world_state->ball;

    const RJ::Time curTime = RJ::now();

    if (previous.empty()) {
        state = LineKickStates::Approach;
    }

    if (state == LineKickStates::FollowThrough) {
        if (!previous.empty()) {
            // Create the actual plan. This will be followed for the rest of the
            // follow-through maneuver.

            Trajectory result = CreatePath::simple(startInstant, )
        }

        auto timeLeft = previous.end_time() - curTime;

        if (timeLeft < RJ::Seconds(-0.3) || timeLeft > RJ::Seconds(5.0)) {
            state = LineKickStates::Approach;
            previous = Trajectory();
        } else {
            return reuse(RJ::now(), request.start, previous);
        }
    }

    // If we get this far then we're in Approach

    bool ball_slow = ball.velocity.mag() < 0.2;

    Trajectory ball_trajectory;
    ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    FillObstacles(request, &static_obstacles, &dynamic_obstacles, !ball_slow, &ball_trajectory);

    // The ball is moving slowly so we can pretty much treat it as stationary
    if (ball_slow) {
        return planForSlowMovingBall(request.start, ball, aimTarget,
                                     std::move(static_obstacles),
                                     dynamic_obstacles, request.constraints);
    }

    if (!prevTrajectory.empty() && _targetKickPos) {
        RJ::Seconds timeInto = RJ::now() - prevTrajectory.begin_time();
        RJ::Seconds timeLeft = prevTrajectory.duration() - timeInto;

        Point targetPos;
        RJ::Time time = ball.query_time_at(*_targetKickPos, &targetPos);
        RJ::Seconds timeToHit = time - curTime;
        if (timeLeft < RJ::Seconds(1.0)) {
            Point targetVel =
                (aimTarget - targetPos).normalized(*_approachSpeed);
            targetPos -= targetVel.normalized(Robot_Radius + Ball_Radius * 2);

            _finalApproach = true;
            RobotInstant goalInstant{Pose{targetPos, 0}, Twist{targetVel, 0},
                                     RJ::Time{0s}};
            Trajectory path = CreatePath::simple(startInstant, goalInstant,
                                                 request.constraints.mot);
            if (!path.empty()) {
                path.setDebugText(
                    "FinalPath" + QString::number(path.duration().count()) +
                    " " + QString::number(timeToHit.count()) + " " +
                    QString::number(time.time_since_epoch().count()));
                PlanAngles(path, startInstant, AngleFns::facePoint(aimTarget),
                           rotationConstraints);
                return std::move(path);
            }
        }

        if (!prevTrajectory.hit(static_obstacles, timeInto) &&
            timeLeft - 1000ms < timeToHit && _reusePathCount < 10) {
            _reusePathCount++;
            Point nearPoint;
            prevTrajectory.setDebugText("Reuse prevPath");
            if (ball.query_time_at(prevTrajectory.last().pose.position(),
                                   &nearPoint) >=
                RJ::now() + timeLeft - 1000ms) {
                return reuse(RJ::now(), startInstant, prevTrajectory);
            }
        }
    }

    std::optional<Trajectory> brutePath = attemptBruteForce(request);
    if (brutePath) return std::move(*brutePath);

    Point targetVel = (aimTarget - ball.position).normalized(*_approachSpeed);
    Point targetPos = ball.position - targetVel.normalized(Robot_Radius * 3);

    RobotInstant targetInstant{Pose{targetPos, 0}, Twist{targetVel, 0},
                               RJ::Time{0s}};
    request.motionCommand = PathTargetCommand{targetInstant};
    Trajectory path = _pathTargetPlanner.plan(std::move(request));
    path.setDebugText("Gives ups");
    _targetKickPos = std::nullopt;
    PlanAngles(path, startInstant, AngleFns::facePoint(aimTarget),
               rotationConstraints);
    return std::move(path);
#endif
    return Trajectory();
}

Trajectory LineKickPlanner::planForSlowMovingBall(
    RobotInstant /* start */, BallState /* ball */,
    Geometry2d::Point /* target */,
    const Geometry2d::ShapeSet& /* static_obstacles */,
    const std::vector<DynamicObstacle>& /* dynamic_obstacles */,
    RobotConstraints /* constraints */) {
#if 0
    constexpr double ballAvoidDistance = 0.05;

    RobotInstant startInstant = start;
    RotationConstraints rotationConstraints = constraints.rot;

    Point targetVel = (target - ball.position).normalized(*_approachSpeed);
    Point targetPos;

    // Calculate angle B'IR, where:
    //  - B: the ball's position
    //  - I: the intercept point
    //  - R: the robot's position
    //  - B': B projected past I
    // B
    // |
    // |
    // I
    // |\
    // | \
    // |  \
    // B'  R
    double robot_capture_angle =
        targetVel.angleBetween(ball.position - startInstant.pose.position());

    if (std::abs(robot_capture_angle) > 10 * M_PI / 180.0) {
        targetPos = ball.position -
                    targetVel.normalized(ballAvoidDistance * 2 + Robot_Radius);
        static_obstacles.add(
            std::make_shared<Circle>(ball.position, ballAvoidDistance));
    } else {
        targetPos = ball.position + targetVel.normalized(Robot_Radius);
        if (!prevTrajectory.empty() &&
            ball.position.distTo(prevTrajectory.last().pose.position()) <
                PathTargetPlanner::goalPosChangeThreshold() &&
            _reusePathCount < 20) {
            targetVel = prevTrajectory.last().velocity.linear();
            _reusePathCount++;
        } else {
            _reusePathCount = 0;
        }
    }
    _targetKickPos = std::nullopt;
    RobotInstant targetInstant{Pose{targetPos, 0}, Twist{targetVel, 0},
                               RJ::Time{0s}};
    request.motionCommand = PathTargetCommand{targetInstant};
    Trajectory path = _pathTargetPlanner.plan(std::move(request));
    PlanAngles(path, startInstant, AngleFns::facePoint(aimTarget),
               rotationConstraints);
    path.setDebugText("Slow Moving Ball");
    return std::move(path);
#endif
    previous = Trajectory();
    return previous;
}

std::optional<Trajectory> LineKickPlanner::attemptBruteForce(
    const PlanRequest& /* request */) {
#if 0
    Trajectory partialPath{{}};
    RJ::Seconds partialPathTime = 0ms;
    RobotInstant tmpStartInstant = request.start;
    const Trajectory& prevTrajectory = request.prevTrajectory;
    BallState ball = request.world_state->ball;
    Point aimTarget = std::get<LineKickCommand>(request.motionCommand).target;
    const double partialReplanLeadTime =
        RRTPlanner::partialReplanLeadTime();
    if (!prevTrajectory.empty()) {
        RJ::Seconds timeInto = RJ::now() - prevTrajectory.begin_time();
        if (timeInto < prevTrajectory.duration() -
                           RJ::Seconds{partialReplanLeadTime * 2}) {
            partialPath = prevTrajectory.subTrajectory(
                0ms, timeInto + RJ::Seconds{partialReplanLeadTime});
            partialPathTime = RJ::Seconds{partialReplanLeadTime};
            tmpStartInstant = partialPath.last();
        }
    }
    RJ::Time curTime = RJ::now();
    for (auto t = RJ::Seconds(0.4); t < RJ::Seconds(6); t += RJ::Seconds(0.2)) {
        Point targetPos = ball.predict_at(curTime + t).position;
        _targetKickPos = targetPos;
        Point targetVel = (aimTarget - targetPos).normalized(*_approachSpeed);
        targetPos -= targetVel.normalized(Robot_Radius + Ball_Radius * 2);
        RobotInstant targetInstant{Pose{targetPos, 0}, Twist{targetVel, 0},
                                   RJ::Time{0s}};
        std::vector<Point> intermediatePoints;
        if (std::abs(targetVel.angleBetween(
                targetPos - tmpStartInstant.pose.position())) > M_PI / 3.0) {
            intermediatePoints.push_back(
                targetPos -
                targetVel.normalized(Robot_Radius * 2.0 + Ball_Radius * 2.0));
        }
        Trajectory path =
            CreatePath::simple(tmpStartInstant, targetInstant,
                               request.constraints.mot, intermediatePoints);
        RJ::Seconds hitTime;
        if (!path.empty()) {
            if (path.duration() + partialPathTime <= t) {
                if (path.hit(request.static_obstacles, RJ::Seconds::zero(),
                             &hitTime)) {
                    continue;
                }
                if (!partialPath.empty()) {
                    path = Trajectory{std::move(partialPath), std::move(path)};
                }
                _reusePathCount = 0;
                PlanAngles(path, request.start, AngleFns::facePoint(aimTarget),
                           request.constraints.rot);
                path.setDebugText("Brute Force");
                return std::move(path);
            }
        }
    }
    return std::nullopt;
#endif
    previous = Trajectory();
    return Trajectory();
}

}  // namespace Planning