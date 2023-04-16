#include "seeker.hpp"

namespace strategy {

Seeker::Seeker(int seeker_num) {
    offense_type_ = "Seeker";
    seeker_pos_ = seeker_num;  // Equal to robot_id
}

std::optional<RobotIntent> Seeker::get_task(RobotIntent intent, const WorldState* world_state,
                                            FieldDimensions field_dimensions) {
    rj_geometry::Point current_loc = world_state->our_robots[intent.robot_id].pose.position();
    rj_geometry::Point target_pt = get_open_point(world_state, current_loc, field_dimensions);

    // stop at end of path
    rj_geometry::Point target_vel{0.0, 0.0};

    planning::PathTargetFaceOption face_option = planning::FaceBall{};

    bool ignore_ball = false;

    planning::LinearMotionInstant goal{target_pt, target_vel};
    intent.motion_command = planning::MotionCommand{"path_target", goal, face_option, ignore_ball};
    return intent;
}

rj_geometry::Point Seeker::get_open_point(const WorldState* world_state,
                                          rj_geometry::Point current_loc,
                                          FieldDimensions field_dimensions) {
    return Seeker::calculate_open_point(1.0, .2, current_loc, world_state, field_dimensions);
}

rj_geometry::Point Seeker::calculate_open_point(double current_prec, double min_prec,
                                                rj_geometry::Point current_point,
                                                const WorldState* world_state,
                                                FieldDimensions field_dimensions) {
    while (current_prec > min_prec) {
        rj_geometry::Point ball_pos = world_state->ball.position;
        rj_geometry::Point min = current_point;
        double min_val = max_los(ball_pos, current_point, world_state);
        double curr_val{};
        // Points in a current_prec radius of the current point, at 45 degree intervals
        std::vector<rj_geometry::Point> check_points{
            correct_point(current_point + rj_geometry::Point{current_prec, 0}, field_dimensions),
            correct_point(current_point + rj_geometry::Point{-current_prec, 0}, field_dimensions),
            correct_point(current_point + rj_geometry::Point{0, current_prec}, field_dimensions),
            correct_point(current_point + rj_geometry::Point{0, -current_prec}, field_dimensions),
            correct_point(
                current_point + rj_geometry::Point{current_prec * 0.707, current_prec * 0.707},
                field_dimensions),
            correct_point(
                current_point + rj_geometry::Point{current_prec * 0.707, -current_prec * 0.707},
                field_dimensions),
            correct_point(
                current_point + rj_geometry::Point{-current_prec * 0.707, current_prec * 0.707},
                field_dimensions),
            correct_point(
                current_point + rj_geometry::Point{-current_prec * 0.707, -current_prec * 0.707},
                field_dimensions)};

        for (auto point : check_points) {
            curr_val = max_los(ball_pos, point + random_noise(current_prec), world_state);
            if (curr_val < min_val) {
                min_val = curr_val;
                min = point;
            }
        }
        current_prec *= 0.5;
        current_point = min;
    }
    return current_point;
}

rj_geometry::Point Seeker::random_noise(double prec) {
    double x = (double)rand() / RAND_MAX * prec;
    double y = (double)rand() / RAND_MAX * prec;
    return rj_geometry::Point{x, y};
}

rj_geometry::Point Seeker::correct_point(rj_geometry::Point p, FieldDimensions field_dimensions) {
    double BORDER_BUFFER = .2;
    double x = p.x();
    double y = p.y();

    // X Border
    if (p.x() > field_dimensions.field_x_right_coord() - BORDER_BUFFER) {
        x = field_dimensions.field_x_right_coord() - BORDER_BUFFER;
    } else if (p.x() < field_dimensions.field_x_left_coord() + BORDER_BUFFER) {
        x = field_dimensions.field_x_left_coord() + BORDER_BUFFER;
    }

    // Y Border
    if (p.y() > field_dimensions.their_goal_loc().y() - BORDER_BUFFER) {
        y = field_dimensions.their_goal_loc().y() - BORDER_BUFFER;
    } else if (p.y() < field_dimensions.our_goal_loc().y() + BORDER_BUFFER) {
        y = field_dimensions.our_goal_loc().y() + BORDER_BUFFER;
    }

    // Goalie Boxes
    if ((y < 1.0 || y > 8.0) && fabs(x) < 1.0) {
        if (y > 4.5) {
            y = 8.0 - BORDER_BUFFER;
        } else {
            y = 1.0 + BORDER_BUFFER;
        }

        if (x > .5) {
            x = 1.0 + BORDER_BUFFER;
        } else {
            x = -1.0 - BORDER_BUFFER;
        }
    }

    // Assigns robots to horizontal thirds
    if (seeker_pos_ % 3 == 0) {
        // Assign left
        if (x > field_dimensions.field_x_left_coord() + field_dimensions.width() / 3) {
            x = field_dimensions.field_x_left_coord() + field_dimensions.width() / 3 -
                BORDER_BUFFER;
        }
    } else if (seeker_pos_ % 3 == 1) {
        // Assign right
        if (x < field_dimensions.field_x_right_coord() - field_dimensions.width() / 3) {
            x = field_dimensions.field_x_right_coord() - field_dimensions.width() / 3 +
                BORDER_BUFFER;
        }
    } else {
        // Assign middle
        if (x < field_dimensions.field_x_left_coord() + field_dimensions.width() / 3) {
            x = field_dimensions.field_x_left_coord() + field_dimensions.width() / 3 +
                BORDER_BUFFER;
        } else if (x > field_dimensions.field_x_right_coord() - field_dimensions.width() / 3) {
            x = field_dimensions.field_x_right_coord() - field_dimensions.width() / 3 -
                BORDER_BUFFER;
        }
    }

    return rj_geometry::Point(x, y);
}

double Seeker::max_los(rj_geometry::Point ball_pos, rj_geometry::Point current_point,
                       const WorldState* world_state) {
    double max = 0;
    double curr_dp;
    for (auto robot : world_state->their_robots) {
        curr_dp = (current_point - ball_pos).norm().dot((robot.pose.position() - ball_pos).norm());
        curr_dp *= curr_dp;
        if (curr_dp > max) {
            max = curr_dp;
        }
    }

    for (auto robot : world_state->our_robots) {
        curr_dp = (current_point - ball_pos).norm().dot((robot.pose.position() - ball_pos).norm());
        curr_dp *= curr_dp;
        if (curr_dp > max) {
            max = curr_dp;
        }
    }

    // Additional heuristics for calculating optimal point
    double ball_proximity_loss = (current_point - ball_pos).mag() * .005;
    double goal_distance_loss = (9.0 - current_point.y()) * .003;

    return max + ball_proximity_loss + goal_distance_loss;
}

}  // namespace strategy