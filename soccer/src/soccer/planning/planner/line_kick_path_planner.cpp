#include "planning/planner/line_kick_path_planner.hpp"

#include <rj_geometry/util.hpp>

#include "control/trapezoidal_motion.hpp"
#include "escape_obstacles_path_planner.hpp"
#include "planning/primitives/create_path.hpp"
#include "planning/trajectory_utils.hpp"

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
        average_ball_vel_ = apply_low_pass_filter(average_ball_vel_, ball.velocity, 0.8);
    }

    process_state_transition();
    switch (current_state_) {
        case INITIAL_APPROACH:
            prev_path_ = initial(plan_request);
            break;
        case FINAL_APPROACH:
            prev_path_ = final(plan_request);
            break;
    }
    prev_path_.stamp(RJ::now());
    return prev_path_;
}

Trajectory LineKickPathPlanner::initial(const PlanRequest& plan_request) {
    // Getting ball info
    const BallState& ball = plan_request.world_state->ball;

    // Distance to stay away from the ball
    auto distance_from_ball = kBallRadius + kRobotRadius + kAvoidBallBy * 4;

    // In case the ball is (slowly) moving
    auto ball_position = ball.predict_at(RJ::now() + RJ::Seconds{kPredictIn}).position;

    // Along the vector from the goal to ball
    auto goal_to_ball = (plan_request.motion_command.target.position - ball_position);
    auto offset_from_ball = distance_from_ball * goal_to_ball.normalized();

    // Create an updated MotionCommand and forward to PathTargetPathPlaner
    PlanRequest modified_request = plan_request;

    LinearMotionInstant target{ball_position - offset_from_ball};

    MotionCommand modified_command{"initial_path_target", target,
                                   FacePoint{plan_request.motion_command.target.position}};
    modified_request.motion_command = modified_command;

    return initial_planner.plan(modified_request);
}

Trajectory LineKickPathPlanner::final(const PlanRequest& plan_request) {
    const BallState& ball = plan_request.world_state->ball;

    // Velocity is the speed (parameter) times the unit vector in the correct direction
    auto goal_to_ball = (plan_request.motion_command.target.position - ball.position);
    auto vel = goal_to_ball.normalized() * kFinalRobotSpeed;

    // Create an updated MotionCommand and forward to PathTargetPathPlaner
    PlanRequest modified_request = plan_request;

    LinearMotionInstant target{ball.position, vel};

    MotionCommand modified_command{"final_path_target", target,
                                   FacePoint{plan_request.motion_command.target.position}};
    modified_request.motion_command = modified_command;

    return final_planner.plan(modified_request);
}

void LineKickPathPlanner::process_state_transition() {
    // Let PathTarget decide when the first stage is done
    // Possible problem: can PathTarget get stuck and loop infinitely?
    if (current_state_ == INITIAL_APPROACH && initial_planner.is_done()) {
        current_state_ = FINAL_APPROACH;
    }
}

bool LineKickPathPlanner::is_done() const {
    // if ball is fast, assume we have kicked it correctly
    // (either way we can't go recapture it)
    return average_ball_vel_.mag() > kIsDoneBallVel;
}

}  // namespace planning
