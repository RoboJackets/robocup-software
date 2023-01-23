#include "defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id) { position_name_ = "Defense"; }

std::optional<RobotIntent> Defense::derived_get_task(RobotIntent intent) {

    // thread-safe getter
    WorldState* world_state = this->world_state();

    // Get Ball Location
    rj_geometry::Point ball_location{world_state->ball.position};

    // Get Goal Location (Always (0,0))
    rj_geometry::Point goal_center_point = {0, 0};

    // Creates Minimum wall radius is slightly greater than  box bounds
    // Dimension accessors should be edited when we figure out how we are doing dimensions realtime
    // from vision
    float box_w{FieldDimensions::kDefaultDimensions.penalty_long_dist()};
    float box_h{FieldDimensions::kDefaultDimensions.penalty_short_dist()};
    float line_w{FieldDimensions::kDefaultDimensions.line_width()};
    double MIN_WALL_RAD{(kRobotRadius * 4.0) + line_w + hypot(box_w / 2, box_h)};

    // Find ball_direction unit vector
    rj_geometry::Point ball_dir_vector = (ball_location - goal_center_point + 0.000001);
    ball_dir_vector /= ball_dir_vector.mag();

    // Find target Point
    rj_geometry::Point mid_point = (goal_center_point) + (ball_dir_vector * MIN_WALL_RAD);

    // Stop at end of path
    rj_geometry::Point target_vel{0.0, 0.0};

    // Face ball
    planning::PathTargetFaceOption face_option = planning::FaceBall{};

    // Avoid ball
    bool ignore_ball = false;

    // Create Motion Command
    planning::LinearMotionInstant goal{mid_point, target_vel};
    intent.motion_command = planning::PathTargetMotionCommand{goal, face_option, ignore_ball};
    intent.motion_command_name = "path_target";
    return intent;
}

}  // namespace strategy
