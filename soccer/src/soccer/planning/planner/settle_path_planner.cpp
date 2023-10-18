#include "settle_path_planner.hpp"

#include <algorithm>
#include <cmath>

#include <spdlog/spdlog.h>

#include <rj_common/utils.hpp>
#include <rj_constants/constants.hpp>

#include "planning/instant.hpp"
#include "planning/primitives/angle_planning.hpp"
#include "planning/primitives/create_path.hpp"
#include "planning/primitives/rrt_util.hpp"
#include "planning/trajectory_utils.hpp"

using namespace rj_geometry;

namespace planning {

Trajectory SettlePathPlanner::plan(const PlanRequest& plan_request) {
    BallState ball = plan_request.world_state.ball;

    const RJ::Time cur_time = plan_request.start.stamp;

    const MotionCommand& command = plan_request.motion_command;

    // The direction we will try and bounce the ball when we dampen it to
    // speed up actions after capture
    target_bounce_direction_ = command.target.position;

    // Start state for the specified robot
    RobotInstant start_instant = plan_request.start;

    bool avoid_ball = true;

    // Smooth out the ball velocity a little bit so we can get a better estimate
    // of intersect points
    if (first_ball_vel_found_) {
        average_ball_vel_ = apply_low_pass_filter<Point>(average_ball_vel_, ball.velocity,
                                                         settle::PARAM_ball_vel_gain);
    } else {
        average_ball_vel_ = ball.velocity;
        first_ball_vel_found_ = true;
    }

    // Figure out where we should place the robot and where to face
    // to get the bounce that we want
    // In the case of no input, it defaults to normal behavior
    double angle = start_instant.heading();
    Point delta_pos;
    Point face_pos;
    calc_delta_pos_for_dir(ball, start_instant, &angle, &delta_pos, &face_pos);

    // Check and see if we should reset the entire thing if we are super far off
    // course or the ball state changes significantly
    check_solution_validity(ball, start_instant, delta_pos);

    if (plan_request.debug_drawer != nullptr) {
        plan_request.debug_drawer->draw_segment(
            Segment(ball.position, ball.position + average_ball_vel_ * 10), QColor(255, 255, 255));
        plan_request.debug_drawer->draw_text("Average Ball Velocity",
                                             ball.position + average_ball_vel_ * 5);
    }

    // Check if we should transition from intercept to dampen
    // Start instant may be changed in that case since we want to start changing
    // the path as soon as possible
    process_state_transition(ball, &start_instant, angle, delta_pos);

    Trajectory result;

    // Run state code
    switch (current_state_) {
        case SettlePathPlannerStates::Intercept:
            result = intercept(plan_request, start_instant, plan_request.static_obstacles, plan_request.dynamic_obstacles,
                               delta_pos, face_pos);
            break;
        case SettlePathPlannerStates::Dampen:
            result = dampen(plan_request, start_instant, delta_pos, face_pos);
            break;
        default:
            result = invalid(plan_request, plan_request.static_obstacles, plan_request.dynamic_obstacles);
            break;
    }

    previous_ = result;
    return result;
}

void SettlePathPlanner::check_solution_validity(BallState ball, RobotInstant start_instant,
                                                rj_geometry::Point delta_pos) {
    const double max_ball_angle_change_for_path_reset =
        settle::PARAM_max_ball_angle_for_reset * M_PI / 180.0f;

    // If the ball changed directions or magnitude really quickly, do a reset of
    // target
    if (average_ball_vel_.angle_between(ball.velocity) > max_ball_angle_change_for_path_reset ||
        (average_ball_vel_ - ball.velocity).mag() > settle::PARAM_max_ball_vel_for_path_reset) {
        first_intercept_target_found_ = false;
        first_ball_vel_found_ = false;
    }

    // Are we too far from the ball line and the ball is still moving
    // or are we too far from a ball not moving towards us
    Line ball_movement_line(ball.position, ball.position + average_ball_vel_);
    Point relative_robot_pos = start_instant.position() - delta_pos;

    bool robot_far = (ball.position - relative_robot_pos).mag() > 2 * kRobotRadius + kBallRadius;
    bool robot_on_ball_line =
        ball_movement_line.dist_to(relative_robot_pos) < kRobotMouthRadius / 2;
    bool ball_moving = average_ball_vel_.mag() > 0.2;
    bool ball_moving_to_us = (ball.position - relative_robot_pos).mag() >
                             (ball.position + 0.01 * average_ball_vel_ - relative_robot_pos).mag();

    if (((!robot_on_ball_line && ball_moving && ball_moving_to_us) ||
         (robot_far && ball_moving && !ball_moving_to_us)) &&
        current_state_ == SettlePathPlannerStates::Dampen) {
        first_intercept_target_found_ = false;
        first_ball_vel_found_ = false;

        current_state_ = SettlePathPlannerStates::Intercept;
    }
}

void SettlePathPlanner::process_state_transition(BallState ball, RobotInstant* start_instant,
                                                 double angle, rj_geometry::Point delta_pos) {
    // State transitions
    // Intercept -> Dampen, PrevPath and almost at the end of the path
    // Dampen -> Complete, PrevPath and almost slowed down to 0?
    if (!previous_.empty() && start_instant->stamp > previous_.begin_time() &&
        start_instant->stamp <= previous_.end_time()) {
        rj_geometry::Line ball_movement_line(ball.position, ball.position + average_ball_vel_);

        Trajectory path_so_far =
            previous_.sub_trajectory(previous_.begin_time(), start_instant->stamp);
        double bot_dist_to_ball_movement_line =
            ball_movement_line.dist_to(path_so_far.last().position() - delta_pos);

        // Intercept -> Dampen
        //  Almost intersecting the ball path and
        //  Almost at end of the target path or
        //  Already in line with the ball
        //
        // TODO(#1518): Check ball sense?

        // Within X seconds of the end of path
        bool inline_with_ball = bot_dist_to_ball_movement_line < cos(angle) * kRobotMouthRadius / 2;
        bool in_front_of_ball =
            average_ball_vel_.angle_between(start_instant->position() - ball.position) < 3.14 / 2;

        if (in_front_of_ball && inline_with_ball &&
            current_state_ == SettlePathPlannerStates::Intercept) {
            // Start the next section of the path from the end of our current
            // path
            *start_instant = path_so_far.last();
            current_state_ = SettlePathPlannerStates::Dampen;
        }
    }
}

Trajectory SettlePathPlanner::intercept(const PlanRequest& plan_request, RobotInstant start_instant,
                                        const rj_geometry::ShapeSet& static_obstacles,
                                        const std::vector<DynamicObstacle>& dynamic_obstacles,
                                        rj_geometry::Point delta_pos, rj_geometry::Point face_pos) {
    BallState ball = plan_request.world_state.ball;

    // Try find best point to intercept using brute force method
    // where we check ever X distance along the ball velocity vector
    //
    // Disallow points outside the field
    const Rect& field_rect = FieldDimensions::current_dimensions.field_rect();

    std::optional<Point> ball_intercept_maybe;
    RJ::Seconds best_buffer = RJ::Seconds(-1.0);

    int num_iterations =
        std::ceil((settle::PARAM_search_end_dist - settle::PARAM_search_start_dist) /
                  settle::PARAM_search_inc_dist);

    for (int iteration = 0; iteration < num_iterations; iteration++) {
        double dist = settle::PARAM_search_start_dist + iteration * settle::PARAM_search_inc_dist;
        // Time for ball to reach the target point
        std::optional<RJ::Seconds> maybe_ball_time = ball.query_seconds_to_dist(dist);

        if (!maybe_ball_time.has_value()) {
            break;
        }

        RJ::Seconds ball_time = maybe_ball_time.value();

        // Account for the target point causing a slight offset in robot
        // position since we want the ball to still hit the mouth
        Point ball_vel_intercept =
            ball.position + average_ball_vel_.normalized() * dist + delta_pos;

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
        Trajectory path = CreatePath::rrt(start_instant.linear_motion(), target_robot_intersection,
                                          plan_request.constraints.mot, start_instant.stamp,
                                          static_obstacles, dynamic_obstacles);

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
        ball_vel_intercept = ball.query_stop_position() + delta_pos;
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
            // Add in the delta_pos for weird target angles since the math is
            // not super fun and not really needed
            ball_vel_intercept = intersect_pts.at(0) + delta_pos;

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
    Point closest_pt = ball_line.nearest_point(start_instant.position()) + delta_pos;

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

        Trajectory shortcut =
            CreatePath::rrt(start_instant.linear_motion(), target, plan_request.constraints.mot,
                            start_instant.stamp, static_obstacles, dynamic_obstacles);

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

    Replanner::PlanParams params{
        start_instant,     target_robot_intersection, static_obstacles,
        dynamic_obstacles, plan_request.constraints,  AngleFns::face_point(face_pos)};
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

Trajectory SettlePathPlanner::dampen(const PlanRequest& plan_request, RobotInstant start_instant,
                                     rj_geometry::Point delta_pos, rj_geometry::Point face_pos) {
    // Only run once if we can

    // Intercept ends with a % ball velocity in the direction of the ball
    // movement Slow down once ball is nearby to 0 m/s

    // Try to slow down as fast as possible to 0 m/s along the ball path
    // We have to do position control since we want to stay in the line of the
    // ball while we do this. If we did velocity, we have very little control of
    // where on the field it is without some other position controller.

    // Uses constant acceleration to create a linear velocity profile

    // TODO(Kyle): Realize the ball will probably bounce off the robot
    // so we can use that vector to stop
    // Save vector and use that?
    BallState ball = plan_request.world_state.ball;

    if (plan_request.debug_drawer != nullptr) {
        plan_request.debug_drawer->draw_text("Damping", ball.position + Point(.1, .1),
                                             QColor(255, 255, 255));
    }

    if (path_created_for_dampen_ && !previous_.empty()) {
        return previous_;
    }

    path_created_for_dampen_ = true;

    if (!previous_.empty()) {
        start_instant = previous_.last();
    }

    // Using the current velocity
    // Calculate stopping point along the ball path
    double max_accel = plan_request.constraints.mot.max_acceleration;
    double current_speed = start_instant.linear_velocity().mag();

    // Assuming const accel going to zero velocity
    // speed / accel gives time to stop
    // speed / 2 is average time over the entire operation
    double stopping_dist = current_speed * current_speed / (2 * max_accel);

    // Offset entire ball line to just be the line we want the robot
    // to move down
    // Accounts for weird targets
    Point ball_movement_dir(average_ball_vel_.normalized());
    Line ball_movement_line(ball.position + delta_pos,
                            ball.position + ball_movement_dir + delta_pos);
    Point nearest_point_to_robot = ball_movement_line.nearest_point(start_instant.position());
    double dist_to_ball_movement_line = (start_instant.position() - nearest_point_to_robot).mag();

    // Default to just moving to the closest point on the line
    Point final_stopping_point(nearest_point_to_robot);

    // Make sure we are actually moving before we start trying to optimize stuff
    if (stopping_dist >= 0.01f) {
        // The closer we are to the line, the less we should move into the line
        // to stop overshoot
        double percent_stopping_dist_to_ball_movement_line =
            dist_to_ball_movement_line / stopping_dist;

        // 0% should be just stopping at stopping_dist down the ball movement
        // line from the nearest_point_to_robot 100% or more should just be trying
        // to get to the nearest_point_to_robot (Default case)
        if (percent_stopping_dist_to_ball_movement_line < 1) {
            // c^2 - a^2 = b^2
            // c is stopping dist, a is dist to ball line
            // b is dist down ball line
            double dist_down_ball_movement_line =
                std::sqrt(stopping_dist * stopping_dist -
                          dist_to_ball_movement_line * dist_to_ball_movement_line);
            final_stopping_point =
                nearest_point_to_robot + dist_down_ball_movement_line * ball_movement_dir;
        }
    }

    // Target stopping point with 0 speed.
    LinearMotionInstant final_stopping_motion{final_stopping_point};

    Trajectory dampen_end = CreatePath::simple(start_instant.linear_motion(), final_stopping_motion,
                                               plan_request.constraints.mot, start_instant.stamp);

    dampen_end.set_debug_text("Damping");

    if (!previous_.empty()) {
        dampen_end = Trajectory(previous_, dampen_end);
    }

    plan_angles(&dampen_end, start_instant, AngleFns::face_point(face_pos),
                plan_request.constraints.rot);
    dampen_end.stamp(RJ::now());
    return dampen_end;
}

Trajectory SettlePathPlanner::invalid(const PlanRequest& plan_request,
                                      const rj_geometry::ShapeSet& static_obstacles,
                                      const std::vector<DynamicObstacle>& dynamic_obstacles) {
    SPDLOG_WARN("Invalid state in settle planner. Restarting");
    current_state_ = SettlePathPlannerStates::Intercept;

    // Stop movement until next frame since it's the safest option
    // programmatically
    LinearMotionInstant target{plan_request.start.position(), Point()};

    Replanner::PlanParams params{
        plan_request.start,       target,
        static_obstacles,         dynamic_obstacles,
        plan_request.constraints, AngleFns::face_point(plan_request.world_state.ball.position)};
    Trajectory path = Replanner::create_plan(params, previous_);
    path.set_debug_text("Invalid state in settle");
    return path;
}

void SettlePathPlanner::calc_delta_pos_for_dir(BallState ball, RobotInstant start_instant,
                                               double* angle_out,
                                               rj_geometry::Point* delta_robot_pos,
                                               rj_geometry::Point* face_pos) {
    // If we have a valid bounce target
    if (target_bounce_direction_) {
        // Get angle between target and normal hit
        Point normal_face_vector = ball.position - start_instant.position();
        Point target_face_vector = *target_bounce_direction_ - start_instant.position();

        // Get the angle between the vectors
        *angle_out = normal_face_vector.angle_between(target_face_vector);

        // Clamp so we don't try to bounce behind us
        *angle_out = std::min(*angle_out, settle::PARAM_max_bounce_angle);

        // Since we loose the sign for the angle between call, there are two
        // possibilities
        Point positive_angle =
            Point(0, -kRobotMouthRadius * sin(*angle_out)).rotate(normal_face_vector.angle());
        Point negative_angle =
            Point(0, kRobotMouthRadius * sin(*angle_out)).rotate(normal_face_vector.angle());

        // Choose the closest one to the true angle
        if (target_face_vector.angle_between(positive_angle) <
            target_face_vector.angle_between(negative_angle)) {
            *delta_robot_pos = negative_angle;
            *face_pos = start_instant.position() +
                        Point::direction(-*angle_out + normal_face_vector.angle()) * 10;
        } else {
            *delta_robot_pos = positive_angle;
            *face_pos = start_instant.position() +
                        Point::direction(*angle_out + normal_face_vector.angle()) * 10;
        }
    } else {
        *delta_robot_pos = Point(0, 0);
        *face_pos = ball.position - average_ball_vel_.normalized();
    }
}

void SettlePathPlanner::reset() {
    current_state_ = SettlePathPlannerStates::Intercept;
    first_intercept_target_found_ = false;
    first_ball_vel_found_ = false;
    path_created_for_dampen_ = false;
    target_bounce_direction_ = std::nullopt;
    previous_ = Trajectory{};
}

bool SettlePathPlanner::is_done() const {
    // FSM: Intercept -> Dampen
    // (see process_state_transition())
    if (current_state_ != SettlePathPlannerStates::Dampen) {
        return false;
    }

    // Dampen is done when the ball is slow-ish, let collect handle actually
    // taking possession
    // TODO(Kevin): consider making settle + collect one planner
    double SETTLE_BALL_SPEED_THRESHOLD = 0.75;
    return average_ball_vel_.mag() < SETTLE_BALL_SPEED_THRESHOLD;
}

}  // namespace planning
