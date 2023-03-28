#include "waller.hpp"

namespace strategy {

Waller::Waller(int waller_num, int total_wallers) {
    defense_type_ = "Waller";
    waller_pos_ = waller_num;
    total_wallers_ = total_wallers;
}

std::optional<RobotIntent> Waller::get_task(RobotIntent intent, const WorldState* world_state) {
    rj_geometry::Point ball_location{world_state->ball.position};

    // Get Goal Location (Always (0,0)) as of creation
    rj_geometry::Point goal_center_point{0, 0};

    // Creates Minimum wall radius is slightly greater than  box bounds
    // Dimension accessors should be edited when we figure out how we are doing dimensions realtime
    // from vision
    float box_w{FieldDimensions::kDefaultDimensions.penalty_long_dist()};
    float box_h{FieldDimensions::kDefaultDimensions.penalty_short_dist()};
    float line_w{FieldDimensions::kDefaultDimensions.line_width()};
    double min_wall_rad{(kRobotRadius * 4.0f) + line_w +
                        hypot(static_cast<double>(box_w) / 2, static_cast<double>((box_h)))};

    // Find ball_direction unit vector
    rj_geometry::Point ball_dir_vector{(ball_location - goal_center_point)};
    // (avoid div by 0)
    ball_dir_vector /= (ball_dir_vector.mag() + 0.000001);

    // Find target Point
    rj_geometry::Point mid_point{(goal_center_point) + (ball_dir_vector * min_wall_rad)};

    // Calculate the wall spacing
    auto wall_spacing = kRobotDiameter + kBallRadius;

    // Calculate the target point
    rj_geometry::Point target_point{};

    if (total_wallers_ % 2 == 0) {
        target_point = mid_point + ball_dir_vector * ((total_wallers_ - waller_pos_ - 0.5) * wall_spacing);
    } else {
        target_point = mid_point + ball_dir_vector * ((total_wallers_ - waller_pos_) * wall_spacing);
    }

    // Stop at end of path
    rj_geometry::Point target_vel{0.0, 0.0};

    // Face ball
    planning::PathTargetFaceOption face_option{planning::FaceBall{}};

    // Avoid ball
    bool ignore_ball{true};

    // Create Motion Command
    planning::LinearMotionInstant target{target_point, target_vel};
    intent.motion_command =
        planning::MotionCommand{"path_target", target, face_option, ignore_ball};
    return intent;
}

}  // namespace strategy
