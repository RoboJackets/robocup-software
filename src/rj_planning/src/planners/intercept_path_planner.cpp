#include "rj_planning/planners/intercept_path_planner.hpp"

#include <rj_common/field_dimensions.hpp>

namespace planning {

Trajectory InterceptPathPlanner::plan(const PlanRequest& plan_request) {
    // Start state for the specified robot
    RobotInstant start_instant = plan_request.start;
    latest_robot_pos_ = start_instant.pose.position();

    BallState ball = plan_request.world_state->ball;
    latest_ball_state_ = ball;

    const rj_geometry::Point robot_pos = start_instant.position();

    // Walk along the ball's future path (from its current position out to where
    // it will stop) and find the earliest point we can beat the ball to.
    // Intercepting as close to the ball's current position as possible gets us
    // in front of the ball sooner.
    const rj_geometry::Point ball_stop_pos = ball.query_stop_position();
    const double ball_path_length = ball.position.dist_to(ball_stop_pos);

    Trajectory trajectory;
    bool found_intercept = false;

    // Only consider intercepting inside our defense area (the goalie box).
    const rj_geometry::Rect goalie_box = plan_request.field_dimensions->our_defense_area();

    if (ball_path_length > 1e-6 && ball.velocity.mag() > 0) {
        const rj_geometry::Point ball_dir = ball.velocity.normalized();

        for (int i = 1; i <= kNumSamples; i++) {
            const double dist_along = ball_path_length * (static_cast<double>(i) / kNumSamples);
            const rj_geometry::Point sample = ball.position + ball_dir * dist_along;

            // Skip points outside the goalie box.
            if (!goalie_box.contains_point(sample) || sample.y() > 0.1) {
                continue;
            }

            // Time for the ball to reach this point along its path.
            const std::optional<RJ::Seconds> ball_time = ball.query_seconds_to_dist(dist_along);
            if (!ball_time.has_value()) {
                continue;
            }

            // Build the trajectory the robot would follow to reach and stop at
            // this point. Its duration accounts for acceleration limits, unlike
            // a naive distance / max_speed estimate.
            Trajectory candidate = CreatePath::simple(
                start_instant.linear_motion(),
                LinearMotionInstant{sample, rj_geometry::Point{0, 0}}, plan_request.constraints.mot,
                start_instant.stamp);

            // First (closest to the ball's start) point we can beat the ball to.
            if (candidate.duration() <= ball_time.value()) {
                trajectory = std::move(candidate);
                found_intercept = true;
                break;
            }
        }
    }

    // If we can't beat the ball to any point along its path, aim for the point
    // on the ball's path closest to the robot (the perpendicular projection).
    if (!found_intercept) {
        rj_geometry::Point intercept_point;
        ball.query_time_near(robot_pos, &intercept_point);
        trajectory = CreatePath::simple(
            start_instant.linear_motion(),
            LinearMotionInstant{intercept_point, rj_geometry::Point{0, 0}},
            plan_request.constraints.mot, start_instant.stamp);
    }

    std::ostringstream debug_text_out;
    debug_text_out.precision(2);
    debug_text_out << (found_intercept ? "Intercept " : "Perp ") << trajectory.duration().count();
    trajectory.set_debug_text(debug_text_out.str());

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
