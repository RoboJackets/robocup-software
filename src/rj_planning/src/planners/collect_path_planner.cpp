#include "rj_planning/planners/collect_path_planner.hpp"

#include <algorithm>
#include <vector>

#include <rj_common/utils.hpp>
#include <rj_param_utils/planning/planning_params.hpp>
#include <spdlog/spdlog.h>

namespace planning {
using namespace rj_geometry;

Trajectory CollectPathPlanner::plan(const PlanRequest& plan_request) {
    const bool previous_ball_sense = is_ball_sense_;

    const auto state = plan_request.play_state.state();
    if (state == PlayState::Stop || state == PlayState::Halt) {
        // This planner automatically fails if the robot is prohibited from touching the ball.
        is_ball_sense_ = false;
        SPDLOG_WARN("Collect planner shell {}: rejected in play state {}.",
                    plan_request.shell_id, static_cast<int>(state));
        return Trajectory{};
    }
    const BallState& ball = plan_request.world_state->ball;

    if (!filtered_ball_velocity_initialized_) {
        filtered_ball_velocity_ = ball.velocity;
        filtered_ball_velocity_initialized_ = true;
    } else {
        filtered_ball_velocity_ = apply_low_pass_filter(
            filtered_ball_velocity_, ball.velocity, collect::PARAM_target_point_lowpass_gain);
    }

    update_state(plan_request, ball.position);
    const Point approach_direction = compute_approach_direction(plan_request.start, ball.position);

    Trajectory path;
    switch (current_state_) {
        case State::APPROACH:
            path = plan_approach(plan_request, approach_direction);
            break;
        case State::CAPTURE:
            path = plan_capture(plan_request, approach_direction);
            break;
    }

    if (path.empty()) {
        is_ball_sense_ = false;
        SPDLOG_WARN("Collect planner shell {}: empty trajectory in state {} (ball_sense={}, "
                    "ball_speed={:.3f}).",
                    plan_request.shell_id,
                    (current_state_ == State::APPROACH ? "APPROACH" : "CAPTURE"),
                    plan_request.ball_sense,
                    ball.velocity.mag());
        if (previous_ball_sense) {
            SPDLOG_INFO("Collect planner shell {}: ball sense FALSE (path empty).",
                        plan_request.shell_id);
        }
        return path;
    }

    path.dribbler_speed = 255;
    is_ball_sense_ = plan_request.ball_sense && current_state_ == State::CAPTURE;

    if (is_ball_sense_ && !previous_ball_sense) {
        SPDLOG_INFO(
            "Collect planner shell {}: ball sense TRUE (sensor {}, state capture).",
            plan_request.shell_id, plan_request.ball_sense);
    } else if (!is_ball_sense_ && previous_ball_sense) {
        SPDLOG_INFO(
            "Collect planner shell {}: ball sense FALSE (sensor {}, state capture {}).",
            plan_request.shell_id, plan_request.ball_sense,
            current_state_ == State::CAPTURE);
    }

    return path;
}

rj_geometry::Point CollectPathPlanner::compute_approach_direction(
    const RobotInstant& start, const Point& ball_position) const {
    const bool ball_is_fast =
        filtered_ball_velocity_.mag() > collect::PARAM_ball_speed_approach_direction_cutoff;

    Point approach_direction = ball_is_fast ? -filtered_ball_velocity_.normalized()
                                            : (ball_position - start.position()).normalized();

    if (approach_direction.mag() < kDirectionEpsilon) {
        if (filtered_ball_velocity_.mag() > kDirectionEpsilon) {
            approach_direction = -filtered_ball_velocity_.normalized();
        } else {
            approach_direction = Point::direction(start.heading());
        }
    }

    return approach_direction;
}

void CollectPathPlanner::update_state(const PlanRequest& request, const Point& ball_position) {
    const double robot_ball_distance = request.start.position().dist_to(ball_position);

    const double capture_enter_distance =
        kRobotMouthRadius + collect::PARAM_approach_dist_target + collect::PARAM_dist_cutoff_to_control;
    const double capture_exit_distance =
        kRobotMouthRadius +
        std::max(capture_enter_distance + kCaptureExitHysteresis,
                 collect::PARAM_dist_cutoff_to_approach);

    if (current_state_ == State::APPROACH && robot_ball_distance <= capture_enter_distance) {
        current_state_ = State::CAPTURE;
    } else if (current_state_ == State::CAPTURE && robot_ball_distance >= capture_exit_distance) {
        current_state_ = State::APPROACH;
    }

    const bool ball_is_fast =
        filtered_ball_velocity_.mag() > collect::PARAM_ball_speed_approach_direction_cutoff;
    if (ball_is_fast &&
        robot_ball_distance > kRobotMouthRadius + collect::PARAM_dist_cutoff_to_control) {
        current_state_ = State::APPROACH;
    }
}

Trajectory CollectPathPlanner::plan_approach(const PlanRequest& request,
                                             const Point& approach_direction) {
    const BallState& ball = request.world_state->ball;

    const Point target_pos =
        ball.position - (kRobotMouthRadius + collect::PARAM_approach_dist_target) * approach_direction;
    const Point target_vel = ball.velocity;

    return build_trajectory(request, target_pos, target_vel, collect::PARAM_approach_accel_scale,
                            "collect_approach");
}

Trajectory CollectPathPlanner::plan_capture(const PlanRequest& request,
                                            const Point& approach_direction) {
    const BallState& ball = request.world_state->ball;

    const Point target_pos = ball.position - kRobotMouthRadius * approach_direction;
    const Point target_vel = ball.velocity + collect::PARAM_touch_delta_speed * approach_direction;

    return build_trajectory(request, target_pos, target_vel, collect::PARAM_control_accel_scale,
                            "collect_capture");
}

Trajectory CollectPathPlanner::build_trajectory(const PlanRequest& request, const Point& target_pos,
                                                const Point& target_vel, double accel_scale,
                                                const char* debug_label) {
    ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    fill_obstacles(request, &static_obstacles, &dynamic_obstacles, false);

    // If we start inside an obstacle, fail and allow fallback planner logic.
    if (static_obstacles.hit(request.start.position())) {
        SPDLOG_WARN("Collect planner shell {}: start in obstacle (state {}, target=({}, {})).",
                    request.shell_id,
                    (current_state_ == State::APPROACH ? "APPROACH" : "CAPTURE"), target_pos.x(),
                    target_pos.y());
        return Trajectory{};
    }

    RobotConstraints constraints = request.constraints;
    constraints.mot.max_acceleration *= accel_scale;

    if (current_state_ == State::CAPTURE) {
        const double capture_speed_limit =
            std::max(target_vel.mag() + collect::PARAM_touch_delta_speed, 0.1);
        constraints.mot.max_speed = std::min(constraints.mot.max_speed, capture_speed_limit);
    }

    LinearMotionInstant target{target_pos, target_vel};

    Replanner::PlanParams params{request.start,
                                 target,
                                 static_obstacles,
                                 dynamic_obstacles,
                                 request.field_dimensions,
                                 constraints,
                                 AngleFns::face_point(request.world_state->ball.position),
                                 request.shell_id};

    Trajectory path = Replanner::create_plan(params, Trajectory{});

    // Capture can become over-constrained near contact. If that happens, retry once
    // with the original max speed before failing out to the default planner.
    if (path.empty() && current_state_ == State::CAPTURE) {
        RobotConstraints relaxed_constraints = constraints;
        relaxed_constraints.mot.max_speed = request.constraints.mot.max_speed;

        Replanner::PlanParams relaxed_params{
            request.start,       target,   static_obstacles, dynamic_obstacles,
            request.field_dimensions, relaxed_constraints,
            AngleFns::face_point(request.world_state->ball.position), request.shell_id};

        path = Replanner::create_plan(relaxed_params, Trajectory{});
        if (!path.empty()) {
            path.set_debug_text("collect_capture_relaxed");
            return path;
        }
    }

    path.set_debug_text(debug_label);
    return path;
}

void CollectPathPlanner::reset() {
    current_state_ = State::APPROACH;
    filtered_ball_velocity_ = Point{0, 0};
    filtered_ball_velocity_initialized_ = false;
    is_ball_sense_ = false;
}

bool CollectPathPlanner::is_done() const { return is_ball_sense_; }

}  // namespace planning
