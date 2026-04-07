#include "rj_planning/planners/line_kick_path_planner.hpp"

using namespace std;
using namespace rj_geometry;

namespace planning {

Trajectory LineKickPathPlanner::plan(const PlanRequest& plan_request) {
    // If we are not allowed to touch the ball, this planner always fails
    // This is preferred to simply ending the planner because it is possible (likely)
    // that strategy re-requests the same planner anyway.
    if (plan_request.play_state == PlayState::halt() ||
        plan_request.play_state == PlayState::stop()) {
        return Trajectory{};
    }

    const BallState& ball = plan_request.world_state->ball;

    if (!average_ball_vel_initialized_) {
        average_ball_vel_ = ball.velocity;
        average_ball_vel_initialized_ = true;
    } else {
        // Add the newest ball velocity measurement to the average velocity
        // estimate, but downweight the new value heavily
        //
        // e.g. new_avg_vel = (0.8 * avg_vel) + (0.2 * new_vel)
        //
        average_ball_vel_ = ball.velocity;
    }

    process_state_transition(plan_request);
    switch (current_state_) {
        case INITIAL_APPROACH:
            prev_path_ = initial(plan_request);
            if (prev_path_.empty()) {
                SPDLOG_INFO("Initial Path Empty!");
            }
            break;
        case FINAL_APPROACH:
            prev_path_ = final(plan_request);
            if (prev_path_.empty()) {
                SPDLOG_INFO("Final Path Empty!");
            }
            break;
    }
    prev_path_.stamp(RJ::now());
    return prev_path_;
}

Trajectory LineKickPathPlanner::initial(const PlanRequest& plan_request) {
    // Getting ball info
    const BallState& ball = plan_request.world_state->ball;

    // Distance to stay away from the ball
    auto distance_from_ball = kBallRadius + kRobotRadius + kAvoidBallBy * 1;

    // In case the ball is (slowly) moving
    auto ball_position = ball.predict_at(RJ::now() + RJ::Seconds{kPredictIn}).position;

    // Along the vector from the goal to ball
    auto goal_to_ball = (plan_request.motion_command.target.position - ball_position);
    auto offset_from_ball = goal_to_ball.normalized(distance_from_ball);

    // Create an updated MotionCommand and forward to PathTargetPathPlaner
    PlanRequest modified_request = plan_request;

    LinearMotionInstant target{ball_position - offset_from_ball};

    MotionCommand modified_command{"path_target", target,
                                   FacePoint{plan_request.motion_command.target.position}};
    modified_request.motion_command = modified_command;

    return path_target_.plan(modified_request);
}

Trajectory LineKickPathPlanner::final(const PlanRequest& plan_request) {
    const BallState& ball = plan_request.world_state->ball;
    if (!prev_path_.empty()) {
        return prev_path_;
    }

    // Velocity is the speed (parameter) times the unit vector in the correct direction
    auto goal_to_ball = (plan_request.motion_command.target.position - ball.position);
    auto vel = goal_to_ball.normalized() * kFinalRobotSpeed;

    // Create an updated MotionCommand and forward to PathTargetPathPlaner
    PlanRequest modified_request = plan_request;

    LinearMotionInstant target{ball.position};
    LinearMotionInstant current = plan_request.start.linear_motion();

    MotionConstraints mot = plan_request.constraints.mot;
    mot.max_speed *= 0.6;
    auto traj = CreatePath::simple(current, target, mot, plan_request.start.stamp);
    plan_angles(&traj, plan_request.start,
                AngleFns::face_point(plan_request.motion_command.target.position),
                plan_request.constraints.rot);
    traj.stamp(RJ::now());

    return traj;
}

void LineKickPathPlanner::process_state_transition(const PlanRequest& plan_request) {
    // Let PathTarget decide when the first stage is done
    // Possible problem: can PathTarget get stuck and loop infinitely?
    auto distance_from_ball = kBallRadius + kRobotRadius + kAvoidBallBy * 4;
    auto ball = plan_request.world_state->ball.position;
    auto us = plan_request.world_state->get_robot(true, plan_request.shell_id).pose.position();
    if (current_state_ == INITIAL_APPROACH && (path_target_.is_done())) {
        current_state_ = FINAL_APPROACH;
        prev_path_ = Trajectory{};
    }

    auto us_to_ball = us - ball;
    auto ball_to_goal = ball - plan_request.motion_command.target.position;
    auto projection = (us_to_ball.dot(ball_to_goal) / ball_to_goal.dot(ball_to_goal));
    us_to_ball = us_to_ball - (projection)*ball_to_goal;

    if (current_state_ == FINAL_APPROACH &&
        (us_to_ball.mag() > kRobotRadius || us.dist_to(ball) > distance_from_ball)) {
        current_state_ = INITIAL_APPROACH;
    }
}

bool LineKickPathPlanner::is_done() const {
    // if ball is fast, assume we have kicked it correctly
    // (either way we can't go recapture it)
    return average_ball_vel_.mag() > kIsDoneBallVel;
}

}  // namespace planning
