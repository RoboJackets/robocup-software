#include "waller_path_planner.hpp"

namespace planning {
using namespace rj_geometry;

Trajectory WallerPathPlanner::plan(const PlanRequest& request) {
    auto command = request.motion_command;
    
    auto robot_id = request.shell_id;
    auto robot_pos = request.world_state->get_robot(true, robot_id).pose.position();
    auto ball_pos = request.world_state->ball.position;
    auto waller_radius = command.waller_radius;
    auto goal_pos = Point{0, 0};
    auto num_wallers = command.num_wallers;
    auto parent_point = request.world_state->get_robot(true, command.waller_parent).pose.position();

    auto waller_id = command.waller_id;

    // Find ball_direction unit vector
    rj_geometry::Point ball_dir_vector{(ball_pos - goal_pos)};

    ball_dir_vector = ball_dir_vector.normalized();

    // Find target Point
    rj_geometry::Point mid_point{(goal_pos) + (ball_dir_vector * waller_radius)};

    auto wall_spacing = kRobotDiameterMultiplier * kRobotDiameter + kBallRadius;

    rj_geometry::Point target_point{};
    SPDLOG_INFO("Radius: {}", waller_radius);
    SPDLOG_INFO("Midpoint: ({}, {})", mid_point.x(), mid_point.y());

    if (abs(robot_pos.dist_to(goal_pos) - waller_radius) > kRobotRadius) {
        auto angle = (mid_point - goal_pos).angle();
        auto delta_angle = (wall_spacing * abs(waller_id - num_wallers / 2. + 0.5)) / waller_radius;
        auto target_angle = angle - delta_angle * (signbit(waller_id - num_wallers / 2. + 0.5) ? -1 : 1);

        target_point = (goal_pos + Point{1, 0}).normalized(waller_radius).rotated(target_angle);

    } else if (ball_pos.x() < robot_pos.x()) {
        if (waller_id == 1) {
            auto angle = (mid_point - goal_pos).angle();
            auto delta_angle = (wall_spacing * abs(waller_id - num_wallers / 2. + 0.5)) / waller_radius;
            auto target_angle = angle - delta_angle * (signbit(waller_id - num_wallers / 2. + 0.5) ? -1 : 1);

            target_point = (goal_pos + Point{1, 0}).normalized(waller_radius).rotated(target_angle);


            SPDLOG_INFO("Target: ({}, {})", target_point.x(), target_point.y());
        } else {
            auto angle = (parent_point - goal_pos).angle();
            auto delta_angle = wall_spacing / waller_radius;
            auto target_angle = angle - delta_angle;

            target_point = (goal_pos + Point{1, 0}).normalized(waller_radius).rotated(target_angle);
        }
    } else {
        if (waller_id == num_wallers) {
            auto angle = (mid_point - goal_pos).angle();
            auto delta_angle = (wall_spacing * abs(waller_id - num_wallers / 2. + 0.5)) / waller_radius;
            auto target_angle = angle - delta_angle * (signbit(waller_id - num_wallers / 2. + 0.5) ? -1 : 1);

            target_point = (goal_pos + Point{1, 0}).normalized(waller_radius).rotated(target_angle);
        } else {
            auto angle = (parent_point - goal_pos).angle();
            auto delta_angle = wall_spacing / waller_radius;
            auto target_angle = angle + delta_angle;

            target_point = (goal_pos + Point{1, 0}).normalized(waller_radius).rotated(target_angle);
        }
    }

    PlanRequest modified_request = request;
    LinearMotionInstant target{target_point};

    MotionCommand modified_command{"path_target", target};
    modified_request.motion_command = modified_command;
    
    return path_target_.plan(modified_request);
}

bool WallerPathPlanner::is_done() const {
    return false;
}

}