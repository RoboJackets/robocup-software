#include "LineKickPlanner.hpp"

#include <Configuration.hpp>
#include <Geometry2d/Util.hpp>
#include <motion/TrapezoidalMotion.hpp>

#include "EscapeObstaclesPathPlanner.hpp"
#include "planning/TrajectoryUtils.hpp"
#include "planning/low_level/CreatePath.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

Trajectory LineKickPlanner::plan(const PlanRequest& planRequest) {
    const float ApproachSpeed = 0.25;

    const float ballAvoidDistance = 0.05;

    const auto& command = std::get<LineKickCommand>(planRequest.motionCommand);

    const RobotInstant& startInstant = planRequest.start;
    const auto& motionConstraints = planRequest.constraints.mot;
    const auto& rotationConstraints = planRequest.constraints.rot;
    const auto& ball = planRequest.world_state->ball;

    if (prevPath.empty()) {
        finalApproach = false;
    }

    ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    FillObstacles(planRequest, &static_obstacles, &dynamic_obstacles, false,
                  nullptr);

    auto obstacles_with_ball = static_obstacles;
    const RJ::Time curTime = startInstant.stamp;
    obstacles_with_ball.add(make_shared<Circle>(
        ball.predict_at(curTime).position, ballAvoidDistance));

    if (finalApproach && targetKickPos) {
        RJ::Seconds duration_into_path = curTime - prevPath.begin_time();

        RobotInstant target = prevPath.last();
        RJ::Time time = ball.query_time_near(*targetKickPos);

        auto timeLeft = prevPath.duration() - duration_into_path;

        if (timeLeft < RJ::Seconds(-0.3) || timeLeft > RJ::Seconds(5.0)) {
            finalApproach = false;
            prevPath = {};
        } else if (timeLeft < RJ::Seconds(0)) {
            prevPath.setDebugText("reuse past done " +
                                  std::to_string(timeLeft.count()));
            return prevPath;
        } else {
            RJ::Seconds timeForBall = time - curTime;
            prevPath.ScaleDuration(
                prevPath.duration() * (timeLeft / timeForBall),
                startInstant.stamp);
            prevPath.setDebugText("reuse final slow " +
                                  std::to_string(timeForBall.count()) + " " +
                                  std::to_string(timeLeft.count()));
            prevPath.stamp(RJ::now());
            return prevPath;
        }
    }

    if (ball.velocity.mag() < 0.2) {
        LinearMotionInstant target{
            ball.position,
            (command.target - ball.position).normalized(ApproachSpeed)};

        auto ball_trajectory = ball.make_trajectory();

        Trajectory path;
        if (std::abs(target.velocity.angleBetween(
                (target.position - startInstant.position()))) >
            DegreesToRadians(10)) {
            target.position -= target.velocity.normalized(
                ballAvoidDistance * 2 + Robot_Radius);
            if (!prevPath.empty() &&
                target.position.distTo(prevPath.last().position()) <
                    Replanner::goalPosChangeThreshold() &&
                reusePathCount < 20) {
                target.velocity = prevPath.last().linear_velocity();
                reusePathCount++;
            } else {
                reusePathCount = 0;
            }

            Replanner::PlanParams params{startInstant,
                                         target,
                                         obstacles_with_ball,
                                         dynamic_obstacles,
                                         planRequest.constraints,
                                         AngleFns::facePoint(command.target)};
            path = Replanner::CreatePlan(params, prevPath);
            path.setDebugText("slow ball 1");
        } else {
            if (!prevPath.empty() &&
                target.position.distTo(prevPath.last().position()) <
                    Replanner::goalPosChangeThreshold() &&
                reusePathCount < 20) {
                target.velocity = prevPath.last().linear_velocity();
                reusePathCount++;
            } else {
                reusePathCount = 0;
            }

            target.position += target.velocity.normalized(Robot_Radius);

            Replanner::PlanParams params{startInstant,
                                         target,
                                         static_obstacles,
                                         dynamic_obstacles,
                                         planRequest.constraints,
                                         AngleFns::facePoint(command.target)};
            path = Replanner::CreatePlan(params, prevPath);
            path.setDebugText("slow ball 2");
        }
        targetKickPos = std::nullopt;
        path.stamp(RJ::now());
        prevPath = path;
        return path;
    }

    if (!prevPath.empty() && targetKickPos) {
        auto previous_duration_remaining =
            prevPath.end_time() - startInstant.stamp;

        LinearMotionInstant target;
        RJ::Time intercept_time =
            ball.query_time_near(*targetKickPos, &target.position);
        RJ::Seconds duration_remaining_adjusted =
            previous_duration_remaining - RJ::Seconds(1.0);
        RJ::Time end_time_adjusted = prevPath.end_time() - RJ::Seconds(1.0);
        if (previous_duration_remaining < RJ::Seconds(0.0)) {
            target.velocity =
                (command.target - target.position).normalized(ApproachSpeed);
            target.position -=
                target.velocity.normalized(Robot_Radius + Ball_Radius * 2);

            Replanner::PlanParams params{startInstant,
                                         target,
                                         static_obstacles,
                                         dynamic_obstacles,
                                         planRequest.constraints,
                                         AngleFns::facePoint(command.target)};
            Trajectory path = Replanner::CreatePlan(params, prevPath);

            if (!path.empty()) {
                path.setDebugText(
                    "FinalPath" + std::to_string(path.duration().count()) +
                    " " +
                    std::to_string(
                        RJ::Seconds(intercept_time - startInstant.stamp)
                            .count()) +
                    " " +
                    std::to_string(intercept_time.time_since_epoch().count()));
                path.stamp(RJ::now());
                prevPath = path;
                return path;
            }
        }

        if (prevPath.CheckTime(startInstant.stamp) &&
            !TrajectoryHitsStatic(prevPath, static_obstacles,
                                  startInstant.stamp, nullptr) &&
            end_time_adjusted < intercept_time && reusePathCount < 10) {
            reusePathCount++;
            Point nearPoint;
            prevPath.setDebugText("Reuse prevPath");
            if (ball.query_time_near(prevPath.last().position(), &nearPoint) >=
                end_time_adjusted) {
                return prevPath;
            }
        }
    }

    Trajectory partialPath;
    RJ::Seconds partialPathTime = 0ms;
    auto tmpStartInstant = startInstant;
    const auto partialReplanLeadTime =
        RJ::Seconds(Replanner::partialReplanLeadTime());

    if (!prevPath.empty() && prevPath.CheckTime(startInstant.stamp)) {
        if (startInstant.stamp <
            prevPath.end_time() - partialReplanLeadTime * 2) {
            partialPath = prevPath.subTrajectory(
                startInstant.stamp, startInstant.stamp + partialReplanLeadTime);
            partialPathTime = partialReplanLeadTime;
            tmpStartInstant = partialPath.last();
        }
    }

    for (auto t = RJ::Seconds(0.4); t < RJ::Seconds(6); t += RJ::Seconds(0.2)) {
        RJ::Time rollout_time = curTime + t;

        auto ball_state_predicted = ball.predict_at(rollout_time);
        LinearMotionInstant target{ball_state_predicted.position};
        targetKickPos = target.position;
        target.velocity =
            (command.target - target.position).normalized(ApproachSpeed);
        target.position -=
            target.velocity.normalized(Robot_Radius + Ball_Radius * 2);

        vector<Point> intermediate_points;
        if (std::abs(target.velocity.angleBetween(
                (target.position - tmpStartInstant.position()))) >
            DegreesToRadians(60)) {
            intermediate_points.push_back(
                target.position - target.velocity.normalized(
                                      Robot_Radius * 2.0 + Ball_Radius * 2.0));
        }

        Trajectory path =
            CreatePath::simple(tmpStartInstant.linear_motion(), target,
                               planRequest.constraints.mot,
                               tmpStartInstant.stamp, intermediate_points);

        if (!path.empty()) {
            if (path.duration() + partialPathTime <= t) {
                if (!partialPath.empty()) {
                    path = Trajectory(std::move(partialPath), path);
                }
                PlanAngles(&path, tmpStartInstant,
                           AngleFns::facePoint(target.position),
                           planRequest.constraints.rot);

                path.setDebugText("FoundPath" +
                                  std::to_string(path.duration().count()));
                reusePathCount = 0;
                path.stamp(RJ::now());
                prevPath = path;
                return path;
            }
        }
    }

    auto ball_predicted = ball.predict_at(curTime);
    LinearMotionInstant target{ball_predicted.position};
    target.velocity =
        (command.target - target.position).normalized(ApproachSpeed);
    target.position -= target.velocity.normalized(Robot_Radius * 3);

    auto ballPath = ball.make_trajectory();
    dynamic_obstacles.emplace_back(Ball_Radius, &ballPath);

    Replanner::PlanParams params{startInstant,
                                 target,
                                 static_obstacles,
                                 dynamic_obstacles,
                                 planRequest.constraints,
                                 AngleFns::facePoint(command.target)};
    Trajectory path = Replanner::CreatePlan(params, prevPath);

    path.setDebugText("Approaching cautious");

    targetKickPos = std::nullopt;
    path.stamp(RJ::now());
    prevPath = path;
    return path;
}

}  // namespace Planning
