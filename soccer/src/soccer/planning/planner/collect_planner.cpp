#include "collect_planner.hpp"

#include <spdlog/spdlog.h>

#include <rj_constants/constants.hpp>

#include "configuration.hpp"
#include "planning/instant.hpp"
#include "planning/primitives/angle_planning.hpp"
#include "planning/primitives/create_path.hpp"
#include "planning/primitives/rrt_util.hpp"

using namespace rj_geometry;

namespace Planning {

REGISTER_CONFIGURABLE(CollectPlanner);

ConfigDouble* CollectPlanner::ball_speed_approach_direction_cutoff;
ConfigDouble* CollectPlanner::approach_accel_scale_percent;
ConfigDouble* CollectPlanner::control_accel_scale_percent;
ConfigDouble* CollectPlanner::approach_dist_target;
ConfigDouble* CollectPlanner::touch_delta_speed;
ConfigDouble* CollectPlanner::velocity_control_scale;
ConfigDouble* CollectPlanner::dist_cutoff_to_control;
ConfigDouble* CollectPlanner::vel_cutoff_to_control;
ConfigDouble* CollectPlanner::dist_cutoff_to_approach;
ConfigDouble* CollectPlanner::stop_dist_scale;
ConfigDouble* CollectPlanner::target_point_averaging_gain;

void CollectPlanner::create_configuration(Configuration* cfg) {
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    ball_speed_approach_direction_cutoff =
        new ConfigDouble(cfg, "Capture/Collect/ballSpeedApproachDirectionCutoff", 0.1);
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    approach_accel_scale_percent =
        new ConfigDouble(cfg, "Capture/Collect/approachAccelScalePercent", 0.7);
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    control_accel_scale_percent =
        new ConfigDouble(cfg, "Capture/Collect/controlAccelScalePercent", 0.8);
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    approach_dist_target = new ConfigDouble(cfg, "Capture/Collect/approachDistTarget", 0.04);
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    touch_delta_speed = new ConfigDouble(cfg, "Capture/Collect/touchDeltaSpeed", 0.1);
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    velocity_control_scale = new ConfigDouble(cfg, "Capture/Collect/velocityControlScale", 1);
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    dist_cutoff_to_control = new ConfigDouble(cfg, "Capture/Collect/distCutoffToControl", 0.05);
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    vel_cutoff_to_control = new ConfigDouble(cfg, "Capture/Collect/velCutoffToControl", 1);
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    dist_cutoff_to_approach = new ConfigDouble(cfg, "Capture/Collect/distCutoffToApproach", 0.3);
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    stop_dist_scale = new ConfigDouble(cfg, "Capture/Collect/stopDistScale", 1);
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    target_point_averaging_gain =
        new ConfigDouble(cfg, "Capture/Collect/targetPointAveragingGain", 0.8);
}

Trajectory CollectPlanner::plan(const PlanRequest& plan_request) {
    BallState ball = plan_request.world_state->ball;

    const RJ::Time cur_time = plan_request.start.stamp;

    const auto& command = std::get<CollectCommand>(plan_request.motion_command);

    // Start state for specified robot
    RobotInstant start_instant = plan_request.start;
    RobotInstant partial_start_instant = start_instant;

    // All the max velocity / acceleration constraints for translation /
    // rotation
    RobotConstraints robot_constraints = plan_request.constraints;
    MotionConstraints& motion_constraints = robot_constraints.mot;

    // List of obstacles
    ShapeSet obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    fill_obstacles(plan_request, &obstacles, &dynamic_obstacles, false);

    // The small beginning part of the previous path
    Trajectory partial_path;

    // The is from the original robot position to the ball
    // We only care about the replan lead time from the current position in the
    // path to the intercept point
    RJ::Seconds time_into_previous_path;

    // How much of the future we are devoting to the partial path
    // 0ms unless we have a partial path, then it's a partial_replan_lead_time
    RJ::Seconds partial_path_time = 0ms;

    // How much of the previous path to steal
    const RJ::Seconds partial_replan_lead_time(Replanner::partial_replan_lead_time());

    // Check and see if we should reset the entire thing if we are super far off
    // coarse or the ball state changes significantly
    check_solution_validity(ball, start_instant);

    // Change start instant to be the partial path end instead of the robot
    // current location if we actually have already calculated a path the frame
    // before
    if (!previous_.empty()) {
        time_into_previous_path = cur_time - previous_.begin_time();

        // Make sure we still have time in the path to replan and correct
        // since it's likely that the old path is slightly off
        //
        // ---|----------------|-----------------|
        // TimeNow     EndPartialPrevPath  FinalTargetPoint
        //                     |-----------------|
        //          Amount of the path we can change this iteration
        if (time_into_previous_path < previous_.duration() - 2 * partial_replan_lead_time &&
            time_into_previous_path > 0ms) {
            RJ::Time new_start = previous_.begin_time();
            RJ::Time new_end = new_start + time_into_previous_path + partial_replan_lead_time;
            partial_path = previous_.sub_trajectory(new_start, new_end);
            partial_path_time = partial_path.duration() - time_into_previous_path;
            partial_start_instant = partial_path.last();
        }
    }

    // Initialize the filter to the ball velocity so there's less ramp up
    if (!average_ball_vel_initialized_) {
        average_ball_vel_ = ball.velocity;
        average_ball_vel_initialized_ = true;
    } else {
        average_ball_vel_ = *target_point_averaging_gain * average_ball_vel_ +
                            (1 - *target_point_averaging_gain) * ball.velocity;
    }

    // Approach direction is the direction we move towards the ball and through
    // it
    if (ball.velocity.mag() < *ball_speed_approach_direction_cutoff) {
        // Move directly to the ball
        approach_direction_ = (ball.position - start_instant.position()).norm();
    } else {
        // Approach the ball from behind
        approach_direction_ = -average_ball_vel_.norm();
    }

    // Check if we should transition to control from approach
    process_state_transition(ball, start_instant);

    switch (current_state_) {
        // Moves from the current location to the slow point of approach
        case CourseApproach:
            previous_ = coarse_approach(plan_request, start_instant, obstacles, dynamic_obstacles);
            break;
        // Moves from the slow point of approach to just before point of contact
        case FineApproach:
            previous_ = fine_approach(plan_request, start_instant, obstacles, dynamic_obstacles);
            break;
        // Move through the ball and stop
        case Control:
            previous_ = control(plan_request, partial_start_instant, partial_path, obstacles,
                                dynamic_obstacles);
            break;
        default:
            previous_ = invalid(plan_request, obstacles, dynamic_obstacles);
            break;
    }

    return previous_;
}

void CollectPlanner::check_solution_validity(BallState ball, RobotInstant start) {
    bool near_ball = (ball.position - start.position()).mag() <
                     *dist_cutoff_to_approach + *dist_cutoff_to_control;

    // Check if we need to go back into approach
    //
    // See if we are not near the ball and both almost stopped
    if (!near_ball && current_state_ == Control) {
        current_state_ = CourseApproach;
        approach_direction_created_ = false;
        control_path_created_ = false;
    }
}

void CollectPlanner::process_state_transition(BallState ball, RobotInstant start_instant) {
    // Do the transitions
    double dist = (start_instant.position() - ball.position).mag() - kRobotMouthRadius;
    double speed_diff =
        (start_instant.linear_velocity() - average_ball_vel_).mag() - *touch_delta_speed;

    // If we are in range to the slow dist
    if (dist < *approach_dist_target + kRobotMouthRadius && current_state_ == CourseApproach) {
        current_state_ = FineApproach;
    }

    // If we are close enough to the target point near the ball
    // and almost the same speed we want, start slowing down
    // TODO(#1518): Check for ball sense?
    if (dist < *dist_cutoff_to_control && speed_diff < *vel_cutoff_to_control &&
        current_state_ == FineApproach) {
        current_state_ = Control;
    }
}

Trajectory CollectPlanner::coarse_approach(const PlanRequest& plan_request, RobotInstant start,
                                           const rj_geometry::ShapeSet& static_obstacles,
                                           const std::vector<DynamicObstacle>& dynamic_obstacles) {
    BallState ball = plan_request.world_state->ball;

    // There are two paths that get combined together
    //
    //
    //     |------------------------|-------| (ball)
    // robot position                 slow pt  hit pt
    //
    // Robot position is where we are at now
    // Slow point is where we want to start the const velocity approach
    //     This is due to our acceleration being not exact causing us to
    //     perpetually bump the ball away
    // Hit point is where the robot will touch the ball for the first time

    // The target position shouldn't be the ball, it should be where the mouth
    // is touching the ball

    // Setup targets for path planner
    Point target_slow_pos =
        ball.position - (*approach_dist_target + kRobotMouthRadius) * approach_direction_;
    Point target_slow_vel = average_ball_vel_ + approach_direction_ * *touch_delta_speed;

    // Force the path to use the same target if it doesn't move too much
    if (!path_coarse_target_initialized_ ||
        (path_coarse_target_ - target_slow_pos).mag() >
            (*approach_dist_target - *dist_cutoff_to_control) / 2) {
        path_coarse_target_ = target_slow_pos;
    }

    LinearMotionInstant target_slow{path_coarse_target_, target_slow_vel};

    Replanner::PlanParams params{start,
                                 target_slow,
                                 static_obstacles,
                                 dynamic_obstacles,
                                 plan_request.constraints,
                                 AngleFns::face_point(ball.position)};
    Trajectory coarse_path = Replanner::create_plan(params, previous_);

    if (plan_request.debug_drawer != nullptr) {
        plan_request.debug_drawer->draw_line(
            Segment(start.position(),
                    start.position() + Point::direction(AngleFns::face_point(ball.position)(
                                           start.linear_motion(), start.heading(), nullptr))));
    }

    // Build a path from now to the slow point
    coarse_path.set_debug_text("coarse");

    return coarse_path;
}

Trajectory CollectPlanner::fine_approach(
    const PlanRequest& plan_request, RobotInstant start_instant,
    const rj_geometry::ShapeSet& /* static_obstacles */,
    const std::vector<DynamicObstacle>& /* dynamic_obstacles */) {
    BallState ball = plan_request.world_state->ball;
    RobotConstraints robot_constraints_hit = plan_request.constraints;
    MotionConstraints& motion_constraints_hit = robot_constraints_hit.mot;

    // There are two paths that get combined together
    //
    //
    //     |------------------------|-------| (ball)
    // robot position                 slow pt  hit pt
    //
    // Robot position is where we are at now
    // Slow point is where we want to start the const velocity approach
    //     This is due to our acceleration being not exact causing us to
    //     perpetually bump the ball away
    // Hit point is where the robot will touch the ball for the first time

    // The target position shouldn't be the ball, it should be where the mouth
    // is touching the ball

    // Setup targets for path planner
    Point target_hit_pos = ball.position - kRobotMouthRadius * approach_direction_;
    Point target_hit_vel = average_ball_vel_ + approach_direction_ * *touch_delta_speed;

    LinearMotionInstant target_hit{target_hit_pos, target_hit_vel};

    // Decrease accel at the end so we more smoothly touch the ball
    motion_constraints_hit.max_acceleration *= *approach_accel_scale_percent;
    // Prevent a last minute accel at the end if the approach dist allows for
    // acceleration in the trapezoid
    motion_constraints_hit.max_speed =
        std::min(target_hit_vel.mag(), motion_constraints_hit.max_speed);

    Trajectory path_hit = CreatePath::simple(start_instant.linear_motion(), target_hit,
                                             plan_request.constraints.mot, start_instant.stamp);

    path_hit.set_debug_text("fine");
    plan_angles(&path_hit, start_instant, AngleFns::face_point(ball.position),
                plan_request.constraints.rot);
    path_hit.stamp(RJ::now());

    if (plan_request.debug_drawer != nullptr) {
        plan_request.debug_drawer->draw_line(
            Segment(start_instant.position(),
                    start_instant.position() +
                        Point::direction(AngleFns::face_point(ball.position)(
                            start_instant.linear_motion(), start_instant.heading(), nullptr))));
    }

    return path_hit;
}

Trajectory CollectPlanner::control(const PlanRequest& plan_request, RobotInstant start,
                                   const Trajectory& /* partial_path */,
                                   const rj_geometry::ShapeSet& static_obstacles,
                                   const std::vector<DynamicObstacle>& dynamic_obstacles) {
    BallState ball = plan_request.world_state->ball;
    RobotConstraints robot_constraints = plan_request.constraints;
    MotionConstraints& motion_constraints = robot_constraints.mot;

    // Only plan the path once and run through it
    // Otherwise it will basically push the ball across the field
    if (control_path_created_ && !previous_.empty()) {
        return previous_;
    }

    control_path_created_ = true;

    // Scale the max acceleration so we don't stop too quickly
    // Set the max speed to the current speed so it stays constant as
    //  we touch the ball and allows the dribbler to get some time to
    //  spin it up to speed
    // Make sure we don't go over our current max speed
    // Shouldn't happen (tm)
    //
    // If the ball is moving towards us (like receiving a pass) just move
    // forward at touch_delta_speed
    double current_speed = average_ball_vel_.mag() + *touch_delta_speed;

    double velocity_scale = *velocity_control_scale;

    // Moving at us
    if (average_ball_vel_.angle_between((ball.position - start.position())) > 3.14 / 2) {
        current_speed = *touch_delta_speed;
        velocity_scale = 0;
    }

    motion_constraints.max_acceleration *= *control_accel_scale_percent;
    motion_constraints.max_speed = std::min(current_speed, motion_constraints.max_speed);

    // Using the current velocity
    // Calculate stopping distance given the acceleration
    double max_accel = motion_constraints.max_acceleration;

    double non_zero_vel_time_delta = *approach_dist_target / *touch_delta_speed;

    // Assuming const accel going to zero velocity
    // speed / accel gives time to stop
    // speed / 2 is average speed over entire operation
    double stopping_dist = *approach_dist_target + current_speed * current_speed / (2 * max_accel);

    // Move through the ball some distance
    // The initial part will be at a constant speed, then it will decelerate to
    // 0 m/s
    double dist_from_ball = *stop_dist_scale * stopping_dist;

    Point target_pos = start.position() + dist_from_ball * (ball.position - start.position() +
                                                            velocity_scale * average_ball_vel_ *
                                                                non_zero_vel_time_delta)
                                                               .norm();
    LinearMotionInstant target{target_pos};

    // Try to use the RRTPlanner to generate the path first
    // It reaches the target better for some reason
    std::vector<Point> start_end_points{start.position(), target.position};
    Trajectory path = CreatePath::rrt(start.linear_motion(), target, motion_constraints,
                                      start.stamp, static_obstacles, dynamic_obstacles);

    if (plan_request.debug_drawer != nullptr) {
        plan_request.debug_drawer->draw_line(
            Segment(start.position(), start.position() + (target.position - start.position()) * 10),
            QColor(255, 255, 255), "Control");
    }

    if (path.empty()) {
        return Trajectory{};
    }

    path.set_debug_text("stopping");

    // Make sure that when the path ends, we don't end up spinning around
    // because we hit go past the ball position at the time of path creation
    Point face_pt = start.position() + 10 * (target.position - start.position()).norm();

    plan_angles(&path, start, AngleFns::face_point(face_pt), robot_constraints.rot);

    if (plan_request.debug_drawer != nullptr) {
        plan_request.debug_drawer->draw_line(
            Segment(start.position(),
                    start.position() + Point::direction(AngleFns::face_point(face_pt)(
                                           start.linear_motion(), start.heading(), nullptr))));
    }

    path.stamp(RJ::now());
    return path;
}

Trajectory CollectPlanner::invalid(const PlanRequest& plan_request,
                                   const rj_geometry::ShapeSet& static_obstacles,
                                   const std::vector<DynamicObstacle>& dynamic_obstacles) {
    SPDLOG_WARN("Invalid state in collect planner. Restarting");
    current_state_ = CourseApproach;

    // Stop movement until next frame since it's the safest option
    // programmatically
    LinearMotionInstant target{plan_request.start.position(), Point()};

    Replanner::PlanParams params{
        plan_request.start,       target,
        static_obstacles,         dynamic_obstacles,
        plan_request.constraints, AngleFns::face_point(plan_request.world_state->ball.position)};
    Trajectory path = Replanner::create_plan(params, previous_);
    path.set_debug_text("Invalid state in collect");

    return path;
}

void CollectPlanner::reset() {
    previous_ = Trajectory();
    current_state_ = CollectPathPlannerStates::CourseApproach;
    average_ball_vel_initialized_ = false;
    approach_direction_created_ = false;
    control_path_created_ = false;
    path_coarse_target_initialized_ = false;
}

}  // namespace Planning
