#include "their_side_lineup.hpp"

namespace strategy {

TheirSideLineup::TheirSideLineup(int r_id) : Position(r_id) {
    position_name_ = "Line";
    r_id_ = r_id;
}

std::optional<RobotIntent> TheirSideLineup::derived_get_task(RobotIntent intent) {
    WorldState* world_state = this->world_state();
    int visibleRobots = world_state->our_robots.size();

    rj_geometry::Point target_pt;
    double fieldLength = field_dimensions_.floor_length() / 2;
    double y = field_dimensions_.their_left_goal_post_coordinate().y();
    double x = field_dimensions_.their_left_corner().x();
    // double x = field_dimensions_.our_right_corner().x(); // IF OTHER SIDE
    double finalPos = (y < fieldLength ? 1 : -1) * fieldLength / (visibleRobots - 1) * r_id_ + y;
    target_pt = rj_geometry::Point(x, finalPos);

    // Stop at end of path
    rj_geometry::Point target_vel{0.0, 0.0};

    // Face ball
    planning::PathTargetFaceOption face_option{planning::FaceBall{}};

    // Avoid ball
    bool ignore_ball{false};

    // Create Motion Command
    planning::LinearMotionInstant target{target_pt, target_vel};
    intent.motion_command =
        planning::MotionCommand{"path_target", target, face_option, ignore_ball};
    return intent;
}

}  // namespace strategy
