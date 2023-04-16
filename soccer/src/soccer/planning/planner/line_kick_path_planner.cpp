#include "line_kick_path_planner.hpp"

#include <rj_geometry/util.hpp>

#include "control/trapezoidal_motion.hpp"
#include "escape_obstacles_path_planner.hpp"
#include "planning/primitives/create_path.hpp"
#include "planning/trajectory_utils.hpp"

using namespace std;
using namespace rj_geometry;

namespace planning {

// TODO(Kevin): this planner should respect goal zone obstacles (which would be
// easier with composability)
Trajectory LineKickPathPlanner::plan(const PlanRequest& plan_request) {
    // only plan line kick if not is_done
    if (this->is_done()) {
        return Trajectory{};
    }

    if (!final_approach_) {
        SPDLOG_INFO("no FINAL APPROACH?");
        return get_behind_ball(plan_request);
        // TODO: make it not do anything here correctly, for some reason the
        // old robot intent is not stopping right now
    } else {
        SPDLOG_INFO("FINAL APPROACH?");
        return drive_through_ball(plan_request);
    }
}

Trajectory LineKickPathPlanner::get_behind_ball(const PlanRequest& plan_request) {
    return Trajectory{};
}

Trajectory LineKickPathPlanner::drive_through_ball(const PlanRequest& plan_request) {
    const MotionCommand& command = plan_request.motion_command;

    if (plan_request.virtual_obstacles.hit(plan_request.start.position())) {
        prev_path_ = Trajectory{};
        return prev_path_;
    }

    if (target_kick_pos_.has_value() &&
        command.target.position.dist_to(target_kick_pos_.value()) > 0.1) {
        prev_path_ = Trajectory{};
        target_kick_pos_ = std::nullopt;
    }

    const RobotInstant& start_instant = plan_request.start;
    const auto& motion_constraints = plan_request.constraints.mot;
    const auto& rotation_constraints = plan_request.constraints.rot;
    const auto& ball = plan_request.world_state->ball;

    // track ball velocity to know if done or not
    if (!average_ball_vel_initialized_) {
        average_ball_vel_ = ball.velocity;
        average_ball_vel_initialized_ = true;
    } else {
        // Add the newest ball velocity measurement to the average velocity
        // estimate, but downweight the new value heavily
        //
        // e.g. new_avg_vel = (0.8 * avg_vel) + (0.2 * new_vel)
        //
        // TODO(Kevin): make this gain a ROS param like collect
        average_ball_vel_ = apply_low_pass_filter(average_ball_vel_, ball.velocity, 0.8);
    }

    if (prev_path_.empty()) {
        final_approach_ = false;
    }

    ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    fill_obstacles(plan_request, &static_obstacles, &dynamic_obstacles, false, nullptr);

    auto obstacles_with_ball = static_obstacles;
    const RJ::Time cur_time = start_instant.stamp;
    obstacles_with_ball.add(
        make_shared<Circle>(ball.predict_at(cur_time).position, ball_avoid_distance_));

    LinearMotionInstant target{
        ball.position, (command.target.position - ball.position).normalized(approach_speed_)};

    auto ball_trajectory = ball.make_trajectory();

    Trajectory path;
    if (std::abs(target.velocity.angle_between((target.position - start_instant.position()))) >
        degrees_to_radians(10)) {
        target.position -= target.velocity.normalized(ball_avoid_distance_ * 2 + kRobotRadius);
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
            AngleFns::face_angle(ball.position.angle_to(command.target.position))};
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
                                     AngleFns::face_point(command.target.position)};
        path = Replanner::create_plan(params, prev_path_);
        path.set_debug_text("slow ball 2");
    }
    target_kick_pos_ = command.target.position;
    path.stamp(RJ::now());
    prev_path_ = path;
    return path;
}

bool LineKickPathPlanner::is_done() const {
    // if ball is fast, assume we have kicked it correctly
    // (either way we can't go recapture it)
    return average_ball_vel_.mag() > IS_DONE_BALL_VEL;
}

}  // namespace planning
