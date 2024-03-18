#include "waller.hpp"

namespace strategy {

Waller::Waller(int waller_num, int robot_id, int total_wallers, std::vector<u_int8_t> waller_ids) {
    defense_type_ = "Waller";
    waller_pos_ = waller_num;
    robot_id_ = robot_id;
    total_wallers_ = total_wallers;
    waller_ids_ = waller_ids;
}

std::optional<RobotIntent> Waller::get_task(RobotIntent intent, const WorldState* world_state,
                                            FieldDimensions field_dimensions) {
    rj_geometry::Point ball_location{world_state->ball.position};

    // Get Goal Location (Always (0,0)) as of creation
    rj_geometry::Point goal_center_point{0, 0};

    // Creates Minimum wall radius is slightly greater than  box bounds
    // Dimension accessors should be edited when we figure out how we are doing dimensions realtime
    // from vision
    float box_w{field_dimensions.penalty_long_dist()};
    float box_h{field_dimensions.penalty_short_dist()};
    float line_w{field_dimensions.line_width()};
    double min_wall_rad{(kRobotRadius * 4.0f) + line_w +
                        hypot(static_cast<double>(box_w) / 2, static_cast<double>((box_h)))};

    // Find ball_direction unit vector
    rj_geometry::Point ball_dir_vector{(ball_location - goal_center_point)};
    // (avoid div by 0)
    ball_dir_vector /= (ball_dir_vector.mag() + 0.000001);

    // Find target Point
    rj_geometry::Point mid_point{(goal_center_point) + (ball_dir_vector * min_wall_rad)};

    // Calculate the wall spacing
    auto wall_spacing = robot_diameter_multiplier_ * kRobotDiameter + kBallRadius;

    // Calculate the target point
    rj_geometry::Point target_point{};

    target_point =
        mid_point +
        ball_dir_vector * ((total_wallers_ - waller_pos_ - (total_wallers_ / 2)) * wall_spacing);

    target_point = target_point.rotate(mid_point, M_PI / 2);

    // Stop at end of path
    rj_geometry::Point target_vel{0.0, 0.0};

    // Face ball
    planning::PathTargetFaceOption face_option{planning::FaceBall{}};

    // Avoid ball
    bool ignore_ball{true};

    // find distance to ball
    rj_geometry::Point ball_pt = world_state->ball.position;
    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    double distance_to_ball = robot_position.dist_to(ball_pt);

    // 0.75 or less is the starting value for distance
    bool dont_kick = true;

    // checks to see if ball is within the kicking distance
    if (distance_to_ball < CLEAR_DIST) {
        std::sort(waller_ids_.begin(), waller_ids_.end());
        int median_id = waller_ids_[waller_ids_.size() / 2];

        // if the robot is the median robot id, kick
        // this is just an arbitrary decision that works good enough
        if (robot_id_ == median_id) {
            planning::LinearMotionInstant target{field_dimensions.their_goal_loc()};
            intent.motion_command = planning::MotionCommand{"line_kick", target};

            intent.shoot_mode = RobotIntent::ShootMode::CHIP;
            intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
            intent.kick_speed = 4.0;
            intent.dribbler_speed = 255.0;
            intent.is_active = true;
            dont_kick = false;
        }
    }

    // standard waller behavior
    if (dont_kick) {
        // Create Motion Command
        planning::LinearMotionInstant target{target_point, target_vel};
        intent.motion_command =
            planning::MotionCommand{"path_target", target, face_option, ignore_ball};
    }
    return intent;
}

}  // namespace strategy
