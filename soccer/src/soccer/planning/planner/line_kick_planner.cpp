#include "line_kick_planner.hpp"

#include <rj_geometry/util.hpp>

#include "control/trapezoidal_motion.hpp"
#include "escape_obstacles_path_planner.hpp"
#include "planning/primitives/create_path.hpp"
#include "planning/trajectory_utils.hpp"

using namespace std;
using namespace rj_geometry;

namespace planning {

Trajectory LineKickPlanner::plan(const PlanRequest& plan_request) {
    const float approach_speed = 0.25;

    const float ball_avoid_distance = 0.05;

    const auto& command = std::get<LineKickCommand>(plan_request.motion_command);

    if (plan_request.virtual_obstacles.hit(plan_request.start.position())) {
        prev_path_ = Trajectory{};
        return prev_path_;
    }

    if (target_kick_pos_.has_value() && command.target.dist_to(target_kick_pos_.value()) > 0.1) {
        prev_path_ = Trajectory{};
        target_kick_pos_ = std::nullopt;
    }

    const RobotInstant& start_instant = plan_request.start;
    const auto& motion_constraints = plan_request.constraints.mot;
    const auto& rotation_constraints = plan_request.constraints.rot;
    const auto& ball = plan_request.world_state->ball;

    if (prev_path_.empty()) {
        final_approach_ = false;
    }

    ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    fill_obstacles(plan_request, &static_obstacles, &dynamic_obstacles, false, nullptr);

    auto obstacles_with_ball = static_obstacles;
    const RJ::Time cur_time = start_instant.stamp;
    obstacles_with_ball.add(
        make_shared<Circle>(ball.predict_at(cur_time).position, ball_avoid_distance));

    // This segfaults. Please rewrite the entire thing.
#if 0
    if (final_approach_ && target_kick_pos_) {
        RJ::Seconds duration_into_path = cur_time - prev_path_.begin_time();

        RobotInstant target = prev_path_.last();
        RJ::Time time = ball.query_time_near(*target_kick_pos_);

        auto time_left = prev_path_.duration() - duration_into_path;

        if (time_left < RJ::Seconds(-0.3) || time_left > RJ::Seconds(5.0)) {
            final_approach_ = false;
            prev_path_ = {};
        } else if (time_left < RJ::Seconds(0)) {
            prev_path_.set_debug_text("reuse past done " + std::to_string(time_left.count()));
            return prev_path_;
        } else {
            RJ::Seconds time_for_ball = time - cur_time;
            prev_path_.scale_duration(prev_path_.duration() * (time_left / time_for_ball),
                                      start_instant.stamp);
            prev_path_.set_debug_text("reuse final slow " + std::to_string(time_for_ball.count()) +
                                      " " + std::to_string(time_left.count()));
            prev_path_.stamp(RJ::now());
            return prev_path_;
        }
    }
#endif

    // TODO(Kyle): This is laughably high
    if (ball.velocity.mag() < 10.0) {
        LinearMotionInstant target{ball.position,
                                   (command.target - ball.position).normalized(approach_speed)};

        auto ball_trajectory = ball.make_trajectory();

        Trajectory path;
        if (std::abs(target.velocity.angle_between((target.position - start_instant.position()))) >
            degrees_to_radians(10)) {
            target.position -= target.velocity.normalized(ball_avoid_distance * 2 + kRobotRadius);
            if (!prev_path_.empty() &&
                target.position.dist_to(prev_path_.last().position()) <
                    Replanner::goal_pos_change_threshold() &&
                reuse_path_count_ < 20) {
                target.velocity = prev_path_.last().linear_velocity();
                reuse_path_count_++;
            } else {
                reuse_path_count_ = 0;
            }

            Replanner::PlanParams params{
                start_instant,
                target,
                obstacles_with_ball,
                dynamic_obstacles,
                plan_request.constraints,
                AngleFns::face_angle(ball.position.angle_to(command.target))};
            path = Replanner::create_plan(params, prev_path_);
            path.set_debug_text("slow ball 1");
        } else {
            if (!prev_path_.empty() &&
                target.position.dist_to(prev_path_.last().position()) <
                    Replanner::goal_pos_change_threshold() &&
                reuse_path_count_ < 20) {
                target.velocity = prev_path_.last().linear_velocity();
                reuse_path_count_++;
            } else {
                reuse_path_count_ = 0;
            }

            target.position += target.velocity.normalized(kRobotRadius);

            Replanner::PlanParams params{start_instant,
                                         target,
                                         static_obstacles,
                                         dynamic_obstacles,
                                         plan_request.constraints,
                                         AngleFns::face_point(command.target)};
            path = Replanner::create_plan(params, prev_path_);
            path.set_debug_text("slow ball 2");
        }
        target_kick_pos_ = command.target;
        path.stamp(RJ::now());
        prev_path_ = path;
        return path;
    }

    return Trajectory{};

    if (!prev_path_.empty() && target_kick_pos_) {
        auto previous_duration_remaining = prev_path_.end_time() - start_instant.stamp;

        LinearMotionInstant target;
        RJ::Time intercept_time = ball.query_time_near(*target_kick_pos_, &target.position);
        RJ::Time end_time_adjusted = prev_path_.end_time() - RJ::Seconds(1.0);
        if (previous_duration_remaining < RJ::Seconds(0.0)) {
            target.velocity = (command.target - target.position).normalized(approach_speed);
            target.position -= target.velocity.normalized(kRobotRadius + kBallRadius * 2);

            Replanner::PlanParams params{start_instant,
                                         target,
                                         static_obstacles,
                                         dynamic_obstacles,
                                         plan_request.constraints,
                                         AngleFns::face_point(command.target)};
            Trajectory path = Replanner::create_plan(params, prev_path_);

            if (!path.empty()) {
                path.set_debug_text(
                    "FinalPath" + std::to_string(path.duration().count()) + " " +
                    std::to_string(RJ::Seconds(intercept_time - start_instant.stamp).count()) +
                    " " + std::to_string(intercept_time.time_since_epoch().count()));
                path.stamp(RJ::now());
                prev_path_ = path;
                return path;
            }
        }

        if (prev_path_.check_time(start_instant.stamp) &&
            !trajectory_hits_static(prev_path_, static_obstacles, start_instant.stamp, nullptr) &&
            end_time_adjusted < intercept_time && reuse_path_count_ < 10) {
            reuse_path_count_++;
            Point near_point;
            prev_path_.set_debug_text("Reuse prev_path");
            if (ball.query_time_near(prev_path_.last().position(), &near_point) >=
                end_time_adjusted) {
                return prev_path_;
            }
        }
    }

    Trajectory partial_path;
    RJ::Seconds partial_path_time = 0ms;
    auto tmp_start_instant = start_instant;
    const auto partial_replan_lead_time = RJ::Seconds(Replanner::partial_replan_lead_time());

    if (!prev_path_.empty() && prev_path_.check_time(start_instant.stamp)) {
        if (start_instant.stamp < prev_path_.end_time() - partial_replan_lead_time * 2) {
            partial_path = prev_path_.sub_trajectory(
                start_instant.stamp, start_instant.stamp + partial_replan_lead_time);
            partial_path_time = partial_replan_lead_time;
            tmp_start_instant = partial_path.last();
        }
    }

    for (auto t = RJ::Seconds(0.4); t < RJ::Seconds(6); t += RJ::Seconds(0.2)) {
        RJ::Time rollout_time = cur_time + t;

        auto ball_state_predicted = ball.predict_at(rollout_time);
        LinearMotionInstant target{ball_state_predicted.position};
        target_kick_pos_ = target.position;
        target.velocity = (command.target - target.position).normalized(approach_speed);
        target.position -= target.velocity.normalized(kRobotRadius + kBallRadius * 2);

        vector<Point> intermediate_points;
        if (std::abs(target.velocity.angle_between(
                (target.position - tmp_start_instant.position()))) > degrees_to_radians(60)) {
            intermediate_points.push_back(
                target.position -
                target.velocity.normalized(kRobotRadius * 2.0 + kBallRadius * 2.0));
        }

        Trajectory path = CreatePath::simple(tmp_start_instant.linear_motion(), target,
                                             plan_request.constraints.mot, tmp_start_instant.stamp,
                                             intermediate_points);

        if (!path.empty()) {
            if (path.duration() + partial_path_time <= t) {
                if (!partial_path.empty()) {
                    path = Trajectory(std::move(partial_path), path);
                }
                plan_angles(&path, tmp_start_instant, AngleFns::face_point(target.position),
                            plan_request.constraints.rot);

                path.set_debug_text("FoundPath" + std::to_string(path.duration().count()));
                reuse_path_count_ = 0;
                path.stamp(RJ::now());
                prev_path_ = path;
                return path;
            }
        }
    }

    auto ball_predicted = ball.predict_at(cur_time);
    LinearMotionInstant target{ball_predicted.position};
    target.velocity = (command.target - target.position).normalized(approach_speed);
    target.position -= target.velocity.normalized(kRobotRadius * 3);

    auto ball_path = ball.make_trajectory();
    dynamic_obstacles.emplace_back(kBallRadius, &ball_path);

    Replanner::PlanParams params{start_instant,
                                 target,
                                 static_obstacles,
                                 dynamic_obstacles,
                                 plan_request.constraints,
                                 AngleFns::face_point(command.target)};
    Trajectory path = Replanner::create_plan(params, prev_path_);

    path.set_debug_text("Approaching cautious");

    path.stamp(RJ::now());
    prev_path_ = path;
    return path;
}

bool LineKickPlanner::is_done() const { return false; }

}  // namespace planning
