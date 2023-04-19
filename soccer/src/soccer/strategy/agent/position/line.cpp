#include "line.hpp"

namespace strategy {

Line::Line(int r_id, int scenario) : Position(r_id) {
    position_name_ = "Line";
    r_id_ = r_id;
    scenario_ = scenario;
}

std::optional<RobotIntent> Line::derived_get_task(RobotIntent intent) {
    WorldState* world_state = this->world_state();
    int visibleRobots = world_state->our_robots.size();

    rj_geometry::Point target_pt;
    double finalPos;
    if (scenario_ == 0) {
        auto ball_pos = world_state->ball.position;
        double goal_y = field_dimensions_.our_left_goal_post_coordinate().y();
        // Positive is our_restart, Negative is their_restart
        int dir = field_dimensions_.center_point().y() - ball_pos.y() < 0 ? 1 : -1;
        double fieldWidth = field_dimensions_.floor_border_width();
        double finalY = dir > 0 ? field_dimensions_.our_defense_area().center().y()
                                : field_dimensions_.their_defense_area().center().y();
        finalPos = fieldWidth / (visibleRobots - 1) * r_id_ - visibleRobots / 2 / 3 + ball_pos.x();
        target_pt = rj_geometry::Point(finalPos, finalY);
    } else if (scenario_ == 1) {
        double fieldLength = field_dimensions_.floor_length() / 2;
        double y = field_dimensions_.our_left_goal_post_coordinate().y();
        double x = field_dimensions_.our_left_corner().x();
        // double x = field_dimensions_.our_right_corner().x(); // IF OTHER SIDE
        finalPos = (y < fieldLength ? 1 : -1) * fieldLength / (visibleRobots - 1) * r_id_ + y;
        target_pt = rj_geometry::Point(x, finalPos);
    }

    // Stop at end of path
    rj_geometry::Point target_vel{0.0, 0.0};

    // Face ball
    planning::PathTargetFaceOption face_option{planning::FaceBall{}};

    // Avoid ball
    bool ignore_ball{true};

    // Create Motion Command
    if (world_state->ball.velocity.mag() <= 0.5) {
        planning::LinearMotionInstant target{target_pt, target_vel};
        intent.motion_command =
            planning::MotionCommand{"path_target", target, face_option, ignore_ball};
        return intent;
    }

    auto empty_motion_cmd = planning::MotionCommand{};
    intent.motion_command = empty_motion_cmd;
    return intent;
}

}  // namespace strategy
