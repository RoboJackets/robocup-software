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
    if (plan_request.play_state_ == PlayState::halt() || plan_request.play_state_ == PlayState::stop()) {
        return Trajectory{};
    }

    const BallState& ball = plan_request.world_state->ball;
    // const MotionCommand& command = plan_request.motion_command;
    // const RobotInstant& start_instant = plan_request.start;

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

    // ShapeSet static_obstacles;
    // std::vector<DynamicObstacle> dynamic_obstacles;
    // fill_obstacles(plan_request, &static_obstacles, &dynamic_obstacles, false, nullptr);

    // auto obstacles_with_ball = static_obstacles;
    // const RJ::Time cur_time = start_instant.stamp;
    // obstacles_with_ball.add(
    //     make_shared<Circle>(ball.predict_at(cur_time).position, ball_avoid_distance));

    // // only plan line kick if not is_done
    if (!this->is_done()) {
        state_transition(ball, plan_request.start);
        switch(current_state_) {
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
    }

Trajectory LineKickPathPlanner::initial(const PlanRequest& plan_request) {
    const BallState& ball = plan_request.world_state->ball;
    PlanRequest modified_request = plan_request;
    Point target_pos = ball.position;
    LinearMotionInstant target{target_pos, {0, 0}};
    MotionCommand modified_command{"line_kick", target};
    modified_request.motion_command = modified_command;
    Trajectory path = path_target_.plan(modified_request);
    return path;
}

Trajectory LineKickPathPlanner::final(const PlanRequest& plan_request) {
    return prev_path_;
}

void LineKickPathPlanner::state_transition(BallState ball, RobotInstant start_instant) {
    if (current_state_ == INITIAL_APPROACH && (ball.position - start_instant.position()).mag() < 0.1) {
        current_state_ = FINAL_APPROACH;
    }
}

bool LineKickPathPlanner::is_done() const {
    // if ball is fast, assume we have kicked it correctly
    // (either way we can't go recapture it)
    return average_ball_vel_.mag() > IS_DONE_BALL_VEL;
}

}  // namespace planning
