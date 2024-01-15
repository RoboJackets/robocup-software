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
    // Getting ball info
    const BallState& ball = plan_request.world_state->ball;

    // Creating a modified plan_request to send to PathTargetPlanner
    PlanRequest modified_request = plan_request;

    // Determining where to navigate to
    // Goes to where ball is predicted to be in 0.1 seconds
    // If this prediction is within .1 meters of initial prediction, then the plan is not recalculated
    const RJ::Seconds predictTime(0.1);
    const RJ::Time cur_time = plan_request.start.stamp + predictTime;
    Point ball_prediction = ball.predict_at(cur_time).position;
    if (target_kick_pos_.has_value()) {
        double distanceBetween = (target_kick_pos_.value() - ball_prediction).mag();
        if (distanceBetween > 0.1) {
            target_kick_pos_ = ball_prediction;
        } else {
            return prev_path_;
        }
    } else {
        target_kick_pos_ = ball_prediction;
    }

    // Determining the velocity
    Point approach_direction = ball_prediction - plan_request.start.position();
    double approach_speed = 50.0;
    Point target_vel = (average_ball_vel_ + approach_direction).normalized() * approach_speed;

    // Updating the motion command
    LinearMotionInstant target{target_kick_pos_.value(), target_vel};
    MotionCommand modified_command{"line_kick", target};
    modified_request.motion_command = modified_command;

    // Getting the new path from PathTargetPlanner
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
