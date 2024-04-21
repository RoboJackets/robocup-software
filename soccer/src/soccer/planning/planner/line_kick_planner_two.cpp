#include "planning/planner/line_kick_planner_two.hpp"

#include <rj_geometry/util.hpp>

#include "control/trapezoidal_motion.hpp"
#include "escape_obstacles_path_planner.hpp"
#include "planning/primitives/create_path.hpp"
#include "planning/trajectory_utils.hpp"

using namespace std;
using namespace rj_geometry;

namespace planning {

Trajectory LineKickPlannerTwo::plan(const PlanRequest& plan_request) {
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

    prev_path_ = final(plan_request);

    prev_path_.stamp(RJ::now());
    return prev_path_;
}

Trajectory LineKickPlannerTwo::final(const PlanRequest& plan_request) {
    const BallState& ball = plan_request.world_state->ball;

    // Velocity is the speed (parameter) times the unit vector in the correct direction
    auto goal_to_ball = (plan_request.motion_command.target.position - ball.position);
    auto vel = goal_to_ball.normalized() * kFinalRobotSpeed;

    // Create an updated MotionCommand and forward to PathTargetPathPlaner
    PlanRequest modified_request = plan_request;

    LinearMotionInstant target{ball.position, vel};

    MotionCommand modified_command{"path_target", target,
                                   FacePoint{plan_request.motion_command.target.position}};
    modified_request.motion_command = modified_command;

    return path_target_.plan(modified_request);
}

bool LineKickPlannerTwo::is_done() const {
    // if ball is fast, assume we have kicked it correctly
    // (either way we can't go recapture it)
    return average_ball_vel_.mag() > kIsDoneBallVel;
}
}  // namespace planning
