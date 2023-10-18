#include "intercept_path_planner.hpp"

#include <rj_constants/constants.hpp>

#include "planning/instant.hpp"
#include "planning/primitives/angle_planning.hpp"
#include "planning/primitives/create_path.hpp"

namespace planning {

Trajectory InterceptPathPlanner::plan(const PlanRequest& plan_request) {
    const MotionCommand& command = plan_request.motion_command;

    // Start state for the specified robot
    RobotInstant start_instant = plan_request.start;
    latest_robot_pos_ = start_instant.pose.position();

    // All the max velocity / acceleration constraints for translation /
    // rotation
    const MotionConstraints& motion_constraints = plan_request.constraints.mot;

    BallState ball = plan_request.world_state.ball;
    latest_ball_state_ = ball;

    // Time for ball to hit target point
    // Target point is projected into ball velocity line
    rj_geometry::Point target_pos_on_line;
    RJ::Seconds ball_to_point_time =
        ball.query_seconds_near(command.target.position, &target_pos_on_line);

    // vector from robot to target
    rj_geometry::Point bot_to_target = (target_pos_on_line - start_instant.position());

    // Max speed we can reach given the distance to target and constant
    // acceleration If we don't constrain the speed, there is a velocity
    // discontinuity in the middle of the path
    double max_speed =
        std::min(start_instant.linear_velocity().mag() +
                     sqrt(2 * motion_constraints.max_acceleration * bot_to_target.mag()),
                 motion_constraints.max_speed);

    // Scale the end velocity by % of max velocity to see if we can reach the
    // target at the same time as the ball
    Trajectory trajectory;

    int num_iterations = 20;
    for (int i = 0; i <= num_iterations; i++) {
        double mag = i * 0.05;

        LinearMotionInstant final_stopping_motion{target_pos_on_line,
                                                  mag * max_speed * bot_to_target.normalized()};

        trajectory = CreatePath::simple(start_instant.linear_motion(), final_stopping_motion,
                                        plan_request.constraints.mot, start_instant.stamp);

        // First path where we can reach the point at or before the ball
        // If the end velocity is not 0, you should reach the point as close
        // to the ball time as possible to just ram it
        if (trajectory.duration() <= ball_to_point_time) {
            std::ostringstream debug_text_out;
            debug_text_out.precision(2);
            debug_text_out << "Time " << trajectory.duration().count();
            trajectory.set_debug_text(debug_text_out.str());

            plan_angles(&trajectory, start_instant, AngleFns::face_point(ball.position),
                        plan_request.constraints.rot);
            trajectory.stamp(RJ::now());
            return trajectory;
        }
    }

    // We couldn't get to the target point in time
    // Just give up and do the max velocity across ball velocity
    // Which ends up being the path after the final loop
    trajectory.set_debug_text("GivingUp");

    plan_angles(&trajectory, start_instant, AngleFns::face_point(ball.position),
                plan_request.constraints.rot);
    trajectory.stamp(RJ::now());

    return trajectory;
}

bool InterceptPathPlanner::is_done() const {
    bool ball_is_slow = latest_ball_state_.velocity.mag() < 0.5;  // m/s
    bool ball_is_close = latest_ball_state_.position.dist_to(latest_robot_pos_) <
                         kRobotRadius + kBallRadius + 0.01;  // m

    return ball_is_slow && ball_is_close;
}

}  // namespace planning
