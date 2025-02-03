#include "collect_path_planner.hpp"

#include <spdlog/spdlog.h>

#include <rj_constants/constants.hpp>

#include "planning/instant.hpp"
#include "planning/primitives/angle_planning.hpp"
#include "planning/primitives/create_path.hpp"
#include "planning/primitives/rrt_util.hpp"

using namespace rj_geometry;

namespace planning {

Trajectory CollectPathPlanner::plan(const PlanRequest& plan_request) {
    const auto state = plan_request.play_state.state();
    if (state == PlayState::Stop || state == PlayState::Halt) {
        // This planner automatically fails if the robot is prohibited from touching the ball.
        return Trajectory{};
    }

    BallState ball = plan_request.world_state->ball;

    const RJ::Time cur_time = plan_request.start.stamp;

    const MotionCommand& command = plan_request.motion_command;

    // Start state for specified robot
    RobotInstant start_instant = plan_request.start;
    RobotInstant partial_start_instant = start_instant;

    // All the max velocity / acceleration constraints for translation /
    // rotation
    RobotConstraints robot_constraints = plan_request.constraints;
    MotionConstraints& motion_constraints = robot_constraints.mot;

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
    // TODO(Kevin): add an issue # here
    // TODO: when ball velocity = 0 but ball is not in mouth (due to bouncing), this fails
    if (!average_ball_vel_initialized_) {
        average_ball_vel_ = ball.velocity;
        average_ball_vel_initialized_ = true;
    } else {
        // Add the newest ball velocity measurement to the average velocity
        // estimate, but downweight the new value heavily
        //
        // e.g. new_avg_vel = (0.8 * avg_vel) + (0.2 * new_vel)
        average_ball_vel_ = apply_low_pass_filter(average_ball_vel_, ball.velocity,
                                                  collect::PARAM_target_point_lowpass_gain);
    }

    // Approach direction is the direction we move towards the ball and through
    // it
    if (ball.velocity.mag() < collect::PARAM_ball_speed_approach_direction_cutoff) {
        // Move directly to the ball
        approach_direction_ = (ball.position - start_instant.position()).norm();
    } else {
        // Approach the ball from behind
        approach_direction_ = -average_ball_vel_.norm();
    }

    // Check if we should transition to control from approach
    process_state_transition(plan_request, ball, start_instant);

    // List of obstacles
    ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;

    if (current_state_ == INTERCEPT) {
        fill_obstacles(plan_request, &static_obstacles, &dynamic_obstacles, true);
    } else {
        Trajectory ball;
        fill_obstacles(plan_request, &static_obstacles, &dynamic_obstacles, false, &ball);
    }

    // Return an empty trajectory if the ball is hitting static obstacles
    // or it is in the goalie area.
    // Check the robot for the same conditions.
    if (static_obstacles.hit(start_instant.pose.position())) {
        return Trajectory{};
    }

    switch (current_state_) {
        // Moves from the current location to the slow point of approach
        case COARSE_APPROACH:
            previous_ =
                coarse_approach(plan_request, start_instant, static_obstacles, dynamic_obstacles);
            break;
        // Moves from the slow point of approach to just before point of contact
        case FINE_APPROACH:
            previous_ =
                fine_approach(plan_request, start_instant, static_obstacles, dynamic_obstacles);
            break;
        // Move through the ball and stop
        case CONTROL:
            previous_ = control(plan_request, partial_start_instant, partial_path, static_obstacles,
                                dynamic_obstacles);
            break;
        case INTERCEPT: {
            previous_ = intercept(plan_request, start_instant, static_obstacles, dynamic_obstacles);
            break;
        }
        default:
            previous_ = invalid(plan_request, static_obstacles, dynamic_obstacles);
            break;
    }

    return previous_;
}

void CollectPathPlanner::check_solution_validity(BallState ball, RobotInstant start) {
    bool near_ball = (ball.position - start.position()).mag() <
                     collect::PARAM_dist_cutoff_to_approach + collect::PARAM_dist_cutoff_to_control;

    // Check if we need to go back into approach
    //
    // See if we are not near the ball and both almost stopped
    if (!near_ball && current_state_ == CONTROL) {
        current_state_ = COARSE_APPROACH;
        approach_direction_created_ = false;
        control_path_created_ = false;
    }
}

void CollectPathPlanner::process_state_transition(const PlanRequest& request, BallState ball,
                                                  RobotInstant start_instant) {
    // If the ball is moving, intercept
    // if not, regularly approach
    if (current_state_ == COARSE_APPROACH &&
        average_ball_vel_.mag() > kInterceptVelocityThreshold) {
        current_state_ = INTERCEPT;
    } else if (current_state_ == INTERCEPT &&
               average_ball_vel_.mag() < kInterceptVelocityThreshold) {
        current_state_ = COARSE_APPROACH;
    }

    // Do the transitions
    double dist = (start_instant.position() - ball.position).mag() - kRobotMouthRadius;
    double speed_diff = (start_instant.linear_velocity() - average_ball_vel_).mag() -
                        collect::PARAM_touch_delta_speed;

    // If we are in range to the slow dist
    if (dist < collect::PARAM_approach_dist_target + kRobotMouthRadius &&
        (current_state_ == COARSE_APPROACH || current_state_ == INTERCEPT)) {
        current_state_ = FINE_APPROACH;
    }

    // If the ball gets knocked far away, go back to CoarseApproach
    if (dist > collect::PARAM_approach_dist_target + kRobotMouthRadius &&
        current_state_ == FINE_APPROACH) {
        current_state_ = COARSE_APPROACH;
    }

    // If we are close enough to the target point near the ball
    // and almost the same speed we want, start slowing down
    if (request.ball_sense && current_state_ == FINE_APPROACH) {
        current_state_ = CONTROL;
    }
}

Trajectory CollectPathPlanner::coarse_approach(
    const PlanRequest& plan_request, RobotInstant start,
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
        ball.position -
        (collect::PARAM_approach_dist_target + kRobotMouthRadius) * approach_direction_;
    Point target_slow_vel =
        average_ball_vel_ + approach_direction_ * collect::PARAM_touch_delta_speed;

    // Force the path to use the same target if it doesn't move too much
    if (!path_coarse_target_initialized_ ||
        (path_coarse_target_ - target_slow_pos).mag() >
            (collect::PARAM_approach_dist_target - collect::PARAM_dist_cutoff_to_control) / 2) {
        path_coarse_target_ = target_slow_pos;
    }

    LinearMotionInstant target_slow{path_coarse_target_, target_slow_vel};

    Replanner::PlanParams params{start,
                                 target_slow,
                                 static_obstacles,
                                 dynamic_obstacles,
                                 plan_request.field_dimensions,
                                 plan_request.constraints,
                                 AngleFns::face_point(ball.position),
                                 plan_request.shell_id};

    Trajectory coarse_path = Replanner::create_plan(params, previous_);

    if (plan_request.debug_drawer != nullptr) {
        plan_request.debug_drawer->draw_segment(
            Segment(start.position(),
                    start.position() + Point::direction(AngleFns::face_point(ball.position)(
                                           start.linear_motion(), start.heading(), nullptr))));
    }

    // Build a path from now to the slow point
    coarse_path.set_debug_text("coarse");

    return coarse_path;
}

Trajectory CollectPathPlanner::intercept(const PlanRequest& plan_request,
                                         RobotInstant start_instant,
                                         const rj_geometry::ShapeSet& static_obstacles,
                                         const std::vector<DynamicObstacle>& dynamic_obstacles) {
    const double max_ball_angle_change_for_path_reset =
        settle::PARAM_max_ball_angle_for_reset * M_PI / 180.0f;

    BallState ball = plan_request.world_state->ball;

    rj_geometry::Point face_pos =
        start_instant.position() +
        Point::direction((ball.position - start_instant.position()).angle()) * 10;

    // If the ball changed directions or magnitude really quickly, do a reset of
    // target
    if (average_ball_vel_.angle_between(ball.velocity) > max_ball_angle_change_for_path_reset ||
        (average_ball_vel_ - ball.velocity).mag() > settle::PARAM_max_ball_vel_for_path_reset) {
        first_intercept_target_found_ = false;
        average_ball_vel_initialized_ = false;
    }

    // Try find best point to intercept using brute force method
    // where we check ever X distance along the ball velocity vector
    //
    // Disallow points outside the field
    const Rect& field_rect = FieldDimensions::current_dimensions.field_rect();

    std::optional<Point> ball_intercept_maybe;
    RJ::Seconds best_buffer = RJ::Seconds(-1.0);

    for (double dist = settle::PARAM_search_start_dist; dist < settle::PARAM_search_end_dist;
         dist += settle::PARAM_search_inc_dist) {
        // Time for ball to reach the target point
        std::optional<RJ::Seconds> maybe_ball_time = ball.query_seconds_to_dist(dist);

        if (!maybe_ball_time.has_value()) {
            break;
        }

        RJ::Seconds ball_time = maybe_ball_time.value();

        // Account for the target point causing a slight offset in robot
        // position since we want the ball to still hit the mouth
        Point ball_vel_intercept = ball.position + average_ball_vel_.normalized() * dist;

        if (!field_rect.contains_point(ball_vel_intercept)) {
            break;
        }

        // Use the mouth to center vector, rotate by X degrees
        // Take the delta between old and new mouth vector and move
        // target_robot_intersection by that amount
        // It should be about stopped at that location.
        // Could add a little backwards motion, but it isn't as clean in the
        // planning side
        LinearMotionInstant target_robot_intersection{ball_vel_intercept, Point()};

        // Plan a path from our partial path start location to the intercept
        // test location
        Trajectory path = CreatePath::intermediate(
            start_instant.linear_motion(), target_robot_intersection, plan_request.constraints.mot,
            start_instant.stamp, static_obstacles, dynamic_obstacles, plan_request.field_dimensions,
            plan_request.shell_id);

        // Calculate the
        RJ::Seconds buffer_duration = ball_time - path.duration();
        if (!path.empty() && buffer_duration > best_buffer) {
            ball_intercept_maybe = ball_vel_intercept;
            best_buffer = buffer_duration;
        }

        // If valid path to location
        // and we can reach the target point before ball
        //
        // Don't do the average here so we can project the intercept point
        // inside the field
        if (!path.empty() && best_buffer > RJ::Seconds(settle::PARAM_intercept_buffer_time)) {
            break;
        }
    }

    rj_geometry::Point ball_vel_intercept;
    // If we still haven't found a valid intercept point, just target the stop
    // point.
    if (ball_intercept_maybe.has_value()) {
        ball_vel_intercept = ball_intercept_maybe.value();
    } else {
        ball_vel_intercept = ball.query_stop_position();
    }

    // Make sure target_robot_intersection is inside the field
    // If not, project it into the field
    if (!field_rect.contains_point(ball_vel_intercept)) {
        auto intersect_return = field_rect.intersects(Segment(ball.position, ball_vel_intercept));

        bool valid_intersect = std::get<0>(intersect_return);
        std::vector<Point> intersect_pts = std::get<1>(intersect_return);

        // If the ball intersects the field at some point
        // Just get the intersect point as the new target
        if (valid_intersect) {
            // Sorts based on distance to intercept target
            // The closest one is the intercept point which the ball moves
            // through leaving the field Not the one on the other side of the
            // field
            sort(intersect_pts.begin(), intersect_pts.end(),
                 [ball_vel_intercept](Point a, Point b) {
                     return (a - ball_vel_intercept).mag() < (b - ball_vel_intercept).mag();
                 });

            // Choose a point just inside the field
            ball_vel_intercept = intersect_pts.at(0);

            // Doesn't intersect
            // project the ball into the field
        } else {
            // Simple projection
            ball_vel_intercept.x() = std::max(ball_vel_intercept.x(), (double)field_rect.minx());
            ball_vel_intercept.x() = std::min(ball_vel_intercept.x(), (double)field_rect.maxx());

            ball_vel_intercept.y() = std::max(ball_vel_intercept.y(), (double)field_rect.miny());
            ball_vel_intercept.y() = std::min(ball_vel_intercept.y(), (double)field_rect.maxy());
        }
    }

    // Could not find a valid path that reach the point first
    // Just go for the farthest point and recalc next time
    if (!first_intercept_target_found_) {
        avg_instantaneous_intercept_target_ = ball_vel_intercept;
        path_intercept_target_ = ball_vel_intercept;

        first_intercept_target_found_ = true;
    } else {
        avg_instantaneous_intercept_target_ =
            apply_low_pass_filter<Point>(avg_instantaneous_intercept_target_, ball_vel_intercept,
                                         settle::PARAM_target_point_gain);
    }

    // Shortcuts the crazy path planner to just move into the path of the ball
    // if we are very close Only shortcuts if the target point is further up the
    // path than we are going to hit AKA only shortcut if we have to move
    // backwards along the path to capture the ball
    //
    // Still want to do the math in case the best point changes
    // which happens a lot when the ball is first kicked

    // If we are within a single radius of the ball path
    // and in front of it
    // just move directly to the path location
    Segment ball_line = Segment(
        ball.position, ball.position + average_ball_vel_.norm() * settle::PARAM_search_end_dist);
    Point closest_pt = ball_line.nearest_point(start_instant.position());

    Point ball_to_pt_dir = closest_pt - ball.position;
    bool in_front_of_ball = average_ball_vel_.angle_between(ball_to_pt_dir) < 3.14 / 2;

    // Only force a direct movement if we are within a small range AND
    // we have run the algorithm at least once AND
    // the target point found in the algorithm is further than we are or just
    // about equal
    if (in_front_of_ball &&
        (closest_pt - start_instant.position()).mag() < settle::PARAM_shortcut_dist &&
        first_intercept_target_found_ &&
        (closest_pt - ball.position).mag() -
                (avg_instantaneous_intercept_target_ - ball.position).mag() <
            settle::PARAM_shortcut_dist) {
        LinearMotionInstant target{closest_pt,
                                   settle::PARAM_ball_speed_percent_for_dampen * average_ball_vel_};

        Trajectory shortcut = CreatePath::intermediate(
            start_instant.linear_motion(), target, plan_request.constraints.mot,
            start_instant.stamp, static_obstacles, dynamic_obstacles, plan_request.field_dimensions,
            plan_request.shell_id);

        if (!shortcut.empty()) {
            plan_angles(&shortcut, start_instant, AngleFns::face_point(face_pos),
                        plan_request.constraints.rot);
            shortcut.stamp(RJ::now());
            return shortcut;
        }
    }

    // There's some major problems with repeatedly changing the target for the
    // path planner To alleviate this problem, we only change the target point
    // when it moves over X amount from the previous path target
    //
    // This combined with the shortcut is guaranteed to get in front of the ball
    // correctly If not, add some sort of distance scale that changes based on
    // how close the robot is to the target
    if ((path_intercept_target_ - avg_instantaneous_intercept_target_).mag() > kRobotMouthRadius) {
        path_intercept_target_ = avg_instantaneous_intercept_target_;
    }

    // Build a new path with the target
    // Since the replanner exists, we don't have to deal with partial paths,
    // just use the interface
    LinearMotionInstant target_robot_intersection{
        path_intercept_target_, settle::PARAM_ball_speed_percent_for_dampen * average_ball_vel_};

    Replanner::PlanParams params{start_instant,
                                 target_robot_intersection,
                                 static_obstacles,
                                 dynamic_obstacles,
                                 plan_request.field_dimensions,
                                 plan_request.constraints,
                                 AngleFns::face_point(face_pos),
                                 plan_request.shell_id};
    Trajectory new_target_path = Replanner::create_plan(params, previous_);

    RJ::Seconds time_of_arrival = new_target_path.duration();
    new_target_path.set_debug_text(std::to_string(time_of_arrival.count()) + " s");

    if (new_target_path.empty()) {
        return previous_;
    }

    plan_angles(&new_target_path, start_instant, AngleFns::face_point(face_pos),
                plan_request.constraints.rot);
    new_target_path.stamp(RJ::now());
    return new_target_path;
}

Trajectory CollectPathPlanner::fine_approach(
    const PlanRequest& plan_request, RobotInstant start_instant,
    const rj_geometry::ShapeSet& static_obstacles,
    const std::vector<DynamicObstacle>& dynamic_obstacles) {
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
    Point target_hit_vel =
        average_ball_vel_ + approach_direction_ * collect::PARAM_touch_delta_speed;

    LinearMotionInstant target_hit{target_hit_pos, target_hit_vel};

    // Decrease accel at the end so we more smoothly touch the ball
    motion_constraints_hit.max_acceleration *= collect::PARAM_approach_accel_scale;
    // Prevent a last minute accel at the end if the approach dist allows for
    // acceleration in the trapezoid
    motion_constraints_hit.max_speed =
        std::min(target_hit_vel.mag(), motion_constraints_hit.max_speed);
    Replanner::PlanParams params{start_instant,
                                 target_hit,
                                 static_obstacles,
                                 dynamic_obstacles,
                                 plan_request.field_dimensions,
                                 plan_request.constraints,
                                 AngleFns::face_point(ball.position),
                                 plan_request.shell_id};
    Trajectory path_hit = Replanner::create_plan(params, previous_);
    path_hit.set_debug_text("fine");

    plan_angles(&path_hit, start_instant, AngleFns::face_point(ball.position),
                plan_request.constraints.rot);
    path_hit.stamp(RJ::now());

    if (plan_request.debug_drawer != nullptr) {
        plan_request.debug_drawer->draw_segment(
            Segment(start_instant.position(),
                    start_instant.position() +
                        Point::direction(AngleFns::face_point(ball.position)(
                            start_instant.linear_motion(), start_instant.heading(), nullptr))));
    }

    return path_hit;
}

Trajectory CollectPathPlanner::control(const PlanRequest& plan_request, RobotInstant start,
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
    double current_speed = average_ball_vel_.mag() + collect::PARAM_touch_delta_speed;

    double velocity_scale = collect::PARAM_velocity_control_scale;

    // Moving at us
    if (average_ball_vel_.angle_between((ball.position - start.position())) > 3.14 / 2) {
        current_speed = collect::PARAM_touch_delta_speed;
        velocity_scale = 0;
    }

    motion_constraints.max_acceleration *= collect::PARAM_control_accel_scale;
    motion_constraints.max_speed = std::min(current_speed, motion_constraints.max_speed);

    // Using the current velocity
    // Calculate stopping distance given the acceleration
    double max_accel = motion_constraints.max_acceleration;

    double non_zero_vel_time_delta =
        collect::PARAM_approach_dist_target / collect::PARAM_touch_delta_speed;

    // Assuming const accel going to zero velocity
    // speed / accel gives time to stop
    // speed / 2 is average speed over entire operation
    double stopping_dist =
        collect::PARAM_approach_dist_target + current_speed * current_speed / (2 * max_accel);

    // Move through the ball some distance
    // The initial part will be at a constant speed, then it will decelerate to
    // 0 m/s
    double dist_from_ball = collect::PARAM_stop_dist_scale * stopping_dist;

    Point target_pos = start.position() + dist_from_ball * (ball.position - start.position() +
                                                            velocity_scale * average_ball_vel_ *
                                                                non_zero_vel_time_delta)
                                                               .norm();
    LinearMotionInstant target{target_pos};

    // save for is_done()
    cached_robot_pos_ = start.position();
    cached_ball_pos_ = ball.position;

    // Try to use the RRTPathPlanner to generate the path first
    // It reaches the target better for some reason
    std::vector<Point> start_end_points{start.position(), target.position};

    Replanner::PlanParams params{start,
                                 target,
                                 static_obstacles,
                                 dynamic_obstacles,
                                 plan_request.field_dimensions,
                                 plan_request.constraints,
                                 AngleFns::face_point(ball.position),
                                 plan_request.shell_id};

    Trajectory path = Replanner::create_plan(params, previous_);

    if (plan_request.debug_drawer != nullptr) {
        plan_request.debug_drawer->draw_segment(
            Segment(start.position(), start.position() + (target.position - start.position()) * 10),
            QColor(255, 255, 255));
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
        plan_request.debug_drawer->draw_segment(
            Segment(start.position(),
                    start.position() + Point::direction(AngleFns::face_point(face_pt)(
                                           start.linear_motion(), start.heading(), nullptr))));
    }

    path.stamp(RJ::now());
    return path;
}

Trajectory CollectPathPlanner::invalid(const PlanRequest& plan_request,
                                       const rj_geometry::ShapeSet& static_obstacles,
                                       const std::vector<DynamicObstacle>& dynamic_obstacles) {
    current_state_ = COARSE_APPROACH;

    // Stop movement until next frame since it's the safest option
    // programmatically
    LinearMotionInstant target{plan_request.start.position(), Point()};

    Replanner::PlanParams params{plan_request.start,
                                 target,
                                 static_obstacles,
                                 dynamic_obstacles,
                                 plan_request.field_dimensions,
                                 plan_request.constraints,
                                 AngleFns::face_point(plan_request.world_state->ball.position),
                                 plan_request.shell_id};
    Trajectory path = Replanner::create_plan(params, previous_);
    path.set_debug_text("Invalid state in collect");

    return path;
}

void CollectPathPlanner::reset() {
    previous_ = Trajectory();
    current_state_ = CollectPathPathPlannerStates::COARSE_APPROACH;
    average_ball_vel_initialized_ = false;
    approach_direction_created_ = false;
    control_path_created_ = false;
    path_coarse_target_initialized_ = false;
}

bool CollectPathPlanner::is_done() const {
    // FSM: CoarseApproach -> FineApproach -> Control
    // (see process_state_transition())
    if (current_state_ != CONTROL) {
        return false;
    } else {
        return true;
    }

    // control state is done when the ball is slowed AND in mouth
    // TODO(Kevin): make these into ROS planning params
    double ball_slow_cutoff = 0.01;  // m/s = ~10% robot radius/s
    bool ball_is_slow = average_ball_vel_initialized_ && average_ball_vel_.mag() < ball_slow_cutoff;

    // ideal ball-mouth distance = 1 kRobotRadius + 1 kBallRadius
    // give some leeway with ball_in_mouth_cutoff
    double ball_in_mouth_cutoff = 0.01;
    double dist_to_ball = (cached_robot_pos_.value() - cached_ball_pos_.value()).mag();
    bool ball_in_mouth = dist_to_ball - (kRobotRadius + kBallRadius) < ball_in_mouth_cutoff;

    return ball_is_slow && ball_in_mouth;
}

}  // namespace planning
