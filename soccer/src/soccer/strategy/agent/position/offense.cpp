#include "offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position(r_id) { position_name_ = "Offense"; }

std::optional<RobotIntent> Offense::derived_get_task(RobotIntent intent) {
    rj_geometry::Point target_pt = get_open_point();
    // stop at end of path
    rj_geometry::Point target_vel{0.0, 0.0};

    // face ball on way up, face path on way down
    planning::PathTargetFaceOption face_option = planning::FaceBall{};
    if (kicking_) {
        face_option = planning::FaceTarget{};
    }

    std::cout << "target pt: " << target_pt.x() << "," << target_pt.y() << std::endl;

    // avoid ball
    bool ignore_ball = false;

    planning::LinearMotionInstant goal{target_pt, target_vel};
    intent.motion_command = planning::PathTargetMotionCommand{goal, face_option, ignore_ball};
    intent.motion_command_name = "path_target";
    return intent;
}
/*
 * Gets a point nearby to the robot that the opposing robots have minimal line-of-sight on
 */
rj_geometry::Point Offense::get_open_point() {
    return Offense::calculate_open_point(
        4, 0.0625, this->world_state()->get_robot(true, this->robot_id_).pose.position());
}

/*
 * Iteratively calculates an optimal point with minimal line of sight by focusing on smaller and
 * smaller areas
 */
rj_geometry::Point Offense::calculate_open_point(double current_prec, double min_prec,
                                                 rj_geometry::Point current_point) {
    if (current_prec < min_prec) {
        return current_point;
    }
    WorldState* world_state = this->world_state();
    rj_geometry::Point ball_pos = world_state->ball.position;
    rj_geometry::Point min = current_point;
    int min_val = max_los(ball_pos, current_point, world_state);
    double curr_val;
    //Points in a current_prec radius of the current point, at 45 degree intervals
    std::vector<rj_geometry::Point> check_points(
        {current_point + rj_geometry::Point{current_prec, 0},
         current_point + rj_geometry::Point{-current_prec, 0},
         current_point + rj_geometry::Point{0, current_prec},
         current_point + rj_geometry::Point{0, -current_prec},
         current_point + rj_geometry::Point{current_prec * 0.707, current_prec * 0.707},
         current_point + rj_geometry::Point{current_prec * 0.707, -current_prec * 0.707},
         current_point + rj_geometry::Point{-current_prec * 0.707, current_prec * 0.707},
         current_point + rj_geometry::Point{-current_prec * 0.707, -current_prec * 0.707}});
    for (auto point : check_points) {
        curr_val = max_los(ball_pos, point, world_state);
        if (curr_val < min_val) {
            min_val = curr_val;
            min = point;
        }
    }
    return calculate_open_point(current_prec * 0.5, min_prec, min);
}

/*
 * Calculates the max line-of-sight that any of the opposing robots have on the balls relative to
 * the current_point
 */
int Offense::max_los(rj_geometry::Point ball_pos, rj_geometry::Point current_point,
                     WorldState* world_state) {
    double max = 0;
    double curr_dp;
    for (auto robot : world_state->their_robots) {
        curr_dp = pow((current_point - ball_pos)
                          .normalized()
                          .dot((robot.pose.position() - ball_pos).normalized()),
                      2);
        if (curr_dp > max) {
            max = curr_dp;
        }
    }
    return max;
}

}  // namespace strategy
