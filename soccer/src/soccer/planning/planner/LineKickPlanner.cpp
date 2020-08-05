#include "LineKickPlanner.hpp"

#include <Configuration.hpp>
#include <Geometry2d/Util.hpp>
#include <motion/TrapezoidalMotion.hpp>

#include "EscapeObstaclesPathPlanner.hpp"
#include "planning/TrajectoryUtils.hpp"
#include "planning/primitives/CreatePath.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

Trajectory LineKickPlanner::plan(const PlanRequest& plan_request) {
    const float approach_speed = 0.25;

    const float ball_avoid_distance = 0.05;

    const auto& command = std::get<LineKickCommand>(plan_request.motionCommand);

    const RobotInstant& start_instant = plan_request.start;
    const auto& motion_constraints = plan_request.constraints.mot;
    const auto& rotation_constraints = plan_request.constraints.rot;
    const auto& ball = plan_request.world_state->ball;

    if (prevPath.empty()) {
        finalApproach = false;
    }

    ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    FillObstacles(plan_request, &static_obstacles, &dynamic_obstacles, false,
                  nullptr);

    auto obstacles_with_ball = static_obstacles;
    const RJ::Time cur_time = start_instant.stamp;
    obstacles_with_ball.add(make_shared<Circle>(
        ball.predict_at(cur_time).position, ball_avoid_distance));

    if (finalApproach && targetKickPos) {
        RJ::Seconds duration_into_path = cur_time - prevPath.begin_time();

        RobotInstant target = prevPath.last();
        RJ::Time time = ball.query_time_near(*targetKickPos);

        auto time_left = prevPath.duration() - duration_into_path;

        if (time_left < RJ::Seconds(-0.3) || time_left > RJ::Seconds(5.0)) {
            finalApproach = false;
            prevPath = {};
        } else if (time_left < RJ::Seconds(0)) {
            prevPath.setDebugText("reuse past done " +
                                  std::to_string(time_left.count()));
            return prevPath;
        } else {
            RJ::Seconds time_for_ball = time - cur_time;
            prevPath.ScaleDuration(
                prevPath.duration() * (time_left / time_for_ball),
                start_instant.stamp);
            prevPath.setDebugText("reuse final slow " +
                                  std::to_string(time_for_ball.count()) + " " +
                                  std::to_string(time_left.count()));
            prevPath.stamp(RJ::now());
            return prevPath;
        }
    }

    if (ball.velocity.mag() < 0.2) {
        LinearMotionInstant target{
            ball.position,
            (command.target - ball.position).normalized(approach_speed)};

        auto ball_trajectory = ball.make_trajectory();

        Trajectory path;
        if (std::abs(target.velocity.angleBetween(
                (target.position - start_instant.position()))) >
            DegreesToRadians(10)) {
            target.position -= target.velocity.normalized(
                ball_avoid_distance * 2 + Robot_Radius);
            if (!prevPath.empty() &&
                target.position.distTo(prevPath.last().position()) <
                    Replanner::goalPosChangeThreshold() &&
                reusePathCount < 20) {
                target.velocity = prevPath.last().linear_velocity();
                reusePathCount++;
            } else {
                reusePathCount = 0;
            }

            Replanner::PlanParams params{start_instant,
                                         target,
                                         obstacles_with_ball,
                                         dynamic_obstacles,
                                         plan_request.constraints,
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

            Replanner::PlanParams params{start_instant,
                                         target,
                                         static_obstacles,
                                         dynamic_obstacles,
                                         plan_request.constraints,
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
            prevPath.end_time() - start_instant.stamp;

        LinearMotionInstant target;
        RJ::Time intercept_time =
            ball.query_time_near(*targetKickPos, &target.position);
        RJ::Time end_time_adjusted = prevPath.end_time() - RJ::Seconds(1.0);
        if (previous_duration_remaining < RJ::Seconds(0.0)) {
            target.velocity =
                (command.target - target.position).normalized(approach_speed);
            target.position -=
                target.velocity.normalized(Robot_Radius + Ball_Radius * 2);

            Replanner::PlanParams params{start_instant,
                                         target,
                                         static_obstacles,
                                         dynamic_obstacles,
                                         plan_request.constraints,
                                         AngleFns::facePoint(command.target)};
            Trajectory path = Replanner::CreatePlan(params, prevPath);

            if (!path.empty()) {
                path.setDebugText(
                    "FinalPath" + std::to_string(path.duration().count()) +
                    " " +
                    std::to_string(
                        RJ::Seconds(intercept_time - start_instant.stamp)
                            .count()) +
                    " " +
                    std::to_string(intercept_time.time_since_epoch().count()));
                path.stamp(RJ::now());
                prevPath = path;
                return path;
            }
        }

        if (prevPath.CheckTime(start_instant.stamp) &&
            !TrajectoryHitsStatic(prevPath, static_obstacles,
                                  start_instant.stamp, nullptr) &&
            end_time_adjusted < intercept_time && reusePathCount < 10) {
            reusePathCount++;
            Point near_point;
            prevPath.setDebugText("Reuse prevPath");
            if (ball.query_time_near(prevPath.last().position(), &near_point) >=
                end_time_adjusted) {
                return prevPath;
            }
        }
    }

    Trajectory partial_path;
    RJ::Seconds partial_path_time = 0ms;
    auto tmp_start_instant = start_instant;
    const auto partial_replan_lead_time =
        RJ::Seconds(Replanner::partialReplanLeadTime());

    if (!prevPath.empty() && prevPath.CheckTime(start_instant.stamp)) {
        if (start_instant.stamp <
            prevPath.end_time() - partial_replan_lead_time * 2) {
            partial_path = prevPath.subTrajectory(
                start_instant.stamp,
                start_instant.stamp + partial_replan_lead_time);
            partial_path_time = partial_replan_lead_time;
            tmp_start_instant = partial_path.last();
        }
    }

    for (auto t = RJ::Seconds(0.4); t < RJ::Seconds(6); t += RJ::Seconds(0.2)) {
        RJ::Time rollout_time = cur_time + t;

        auto ball_state_predicted = ball.predict_at(rollout_time);
        LinearMotionInstant target{ball_state_predicted.position};
        targetKickPos = target.position;
        target.velocity =
            (command.target - target.position).normalized(approach_speed);
        target.position -=
            target.velocity.normalized(Robot_Radius + Ball_Radius * 2);

        vector<Point> intermediate_points;
        if (std::abs(target.velocity.angleBetween(
                (target.position - tmp_start_instant.position()))) >
            DegreesToRadians(60)) {
            intermediate_points.push_back(
                target.position - target.velocity.normalized(
                                      Robot_Radius * 2.0 + Ball_Radius * 2.0));
        }

        Trajectory path =
            CreatePath::simple(tmp_start_instant.linear_motion(), target,
                               plan_request.constraints.mot,
                               tmp_start_instant.stamp, intermediate_points);

        if (!path.empty()) {
            if (path.duration() + partial_path_time <= t) {
                if (!partial_path.empty()) {
                    path = Trajectory(std::move(partial_path), path);
                }
                PlanAngles(&path, tmp_start_instant,
                           AngleFns::facePoint(target.position),
                           plan_request.constraints.rot);

                path.setDebugText("FoundPath" +
                                  std::to_string(path.duration().count()));
                reusePathCount = 0;
                path.stamp(RJ::now());
                prevPath = path;
                return path;
            }
        }
    }

    auto ball_predicted = ball.predict_at(cur_time);
    LinearMotionInstant target{ball_predicted.position};
    target.velocity =
        (command.target - target.position).normalized(approach_speed);
    target.position -= target.velocity.normalized(Robot_Radius * 3);

    auto ball_path = ball.make_trajectory();
    dynamic_obstacles.emplace_back(Ball_Radius, &ball_path);

    Replanner::PlanParams params{start_instant,
                                 target,
                                 static_obstacles,
                                 dynamic_obstacles,
                                 plan_request.constraints,
                                 AngleFns::facePoint(command.target)};
    Trajectory path = Replanner::CreatePlan(params, prevPath);

    path.setDebugText("Approaching cautious");

    targetKickPos = std::nullopt;
    path.stamp(RJ::now());
    prevPath = path;
    return path;
}

}  // namespace Planning
