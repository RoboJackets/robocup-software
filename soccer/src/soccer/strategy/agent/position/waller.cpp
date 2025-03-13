#include "waller.hpp"

namespace strategy {

Waller::Waller(int waller_num, std::vector<u_int8_t> walling_robots) {
    defense_type_ = "Waller";
    waller_pos_ = waller_num;
    walling_robots_ = walling_robots;
}

std::optional<RobotIntent> Waller::get_task(RobotIntent intent, const WorldState* world_state,
                                            FieldDimensions field_dimensions) {
    // Creates Minimum wall radius is slightly greater than  box bounds
    // Dimension accessors should be edited when we figure out how we are doing dimensions realtime
    // from vision
    SPDLOG_INFO("In waller::get_task");
    float box_w{field_dimensions.penalty_long_dist()};
    float box_h{field_dimensions.penalty_short_dist()};
    float line_w{field_dimensions.line_width()};
    double min_wall_rad{(kRobotRadius * 4.0f) + line_w +
                        hypot(static_cast<double>(box_w) / 2, static_cast<double>((box_h)))};

    auto ball_pos = world_state->ball.position;

    auto robot_id = intent.robot_id;
    SPDLOG_INFO("Waller checkpoint 1");

    auto robot_pos = world_state->get_robot(true, robot_id).pose.position();
    auto goal_pos = rj_geometry::Point{0, 0};
    auto num_wallers = walling_robots_.size();

    SPDLOG_INFO("Waller Checkpoint 2");
    // Find ball_direction unit vector
    rj_geometry::Point ball_dir_vector{(ball_pos - goal_pos)};

    ball_dir_vector = ball_dir_vector.normalized();

    // Find target Point
    rj_geometry::Point mid_point{(goal_pos) + (ball_dir_vector * min_wall_rad)};

    auto wall_spacing = kRobotDiameterMultiplier * kRobotDiameter + kBallRadius;

    SPDLOG_INFO("Waller Checkpoint 3");
    rj_geometry::Point target_point{};
    auto angle = (mid_point - goal_pos).angle();
    auto delta_angle = (wall_spacing * (waller_pos_ - num_wallers / 2. - 0.5)) / min_wall_rad; // Why are we making this calculation and why change it
    auto target_angle = angle - delta_angle;                                                   // when distance to the goal - min_wall_rad < krobotradius

    target_point =
        (goal_pos + rj_geometry::Point{1, 0}).normalized(min_wall_rad).rotated(target_angle);
    SPDLOG_INFO("Waller Checkpoint 4");

    if (abs(robot_pos.dist_to(goal_pos) - min_wall_rad) < kRobotRadius &&
        robot_pos.dist_to(target_point) > kRobotRadius) {
        uint8_t parent_id;

        if (target_point.x() < robot_pos.x()) {
            parent_id = walling_robots_[waller_pos_ - 2];
        } else {
            parent_id = walling_robots_[waller_pos_];
        }

        if ((target_point.x() < robot_pos.x() && waller_pos_ != 1) ||
            (target_point.x() > robot_pos.x() && waller_pos_ != num_wallers)) {
            auto parent_point = world_state->get_robot(true, parent_id).pose.position(); //we found our crashing line babyyyyyy // TODO: Fix
            angle = (parent_point - goal_pos).angle();
            delta_angle = wall_spacing / min_wall_rad;
            target_angle =
                angle + delta_angle * (signbit(target_point.x() - robot_pos.x()) ? -1 : 1);

            target_point = (goal_pos + rj_geometry::Point{1, 0})
                               .normalized(min_wall_rad)
                               .rotated(target_angle);
        }
    }
    SPDLOG_INFO("Waller Checkpoint 5");

    auto location_instant = planning::LinearMotionInstant{target_point, rj_geometry::Point{}};

    auto target_cmd =
        planning::MotionCommand{"path_target", location_instant, planning::FaceBall{}};
    intent.motion_command = target_cmd;

    SPDLOG_INFO("Waller Checkpoint 6");
    return intent;
}

}  // namespace strategy
