#include "seeking.hpp"

namespace strategy {

Seeking::Seeking(int robot_id) { robot_id_ = robot_id; }

std::optional<RobotIntent> Seeking::get_task(RobotIntent intent, const WorldState* last_world_state,
                                             FieldDimensions field_dimensions) {
    // Determine target position for seeking
    rj_geometry::Point current_loc = last_world_state->get_robot(true, robot_id_).pose.position();

    target_pt = get_open_point(last_world_state, current_loc, field_dimensions);

    planning::PathTargetFaceOption face_option = planning::FaceBall{};
    bool ignore_ball = false;
    planning::LinearMotionInstant goal{target_pt, rj_geometry::Point{0.0, 0.0}};
    intent.motion_command = planning::MotionCommand{"path_target", goal, face_option, ignore_ball};

    return intent;
}

rj_geometry::Point Seeking::get_open_point(const WorldState* world_state,
                                           rj_geometry::Point current_loc,
                                           FieldDimensions field_dimensions) {
    return Seeking::calculate_open_point(3.0, .2, current_loc, world_state, field_dimensions);
}

rj_geometry::Point Seeking::calculate_open_point(double current_prec, double min_prec,
                                                 rj_geometry::Point current_point,
                                                 const WorldState* world_state,
                                                 FieldDimensions field_dimensions) {
    while (current_prec > min_prec) {
        rj_geometry::Point ball_pos = world_state->ball.position;
        rj_geometry::Point min = current_point;
        double min_val = eval_point(ball_pos, current_point, world_state);
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

        // Finds the best point out of the ones checked
        for (auto point : check_points) {
            curr_val = eval_point(ball_pos, point, world_state);
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

rj_geometry::Point Seeking::correct_point(rj_geometry::Point p, FieldDimensions field_dimensions) {
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
    if ((y < 1.2 || y > 7.8) && fabs(x) < 1.2) {
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
    if (robot_id_ == 1) {
        // Assign left
        if (x > field_dimensions.field_x_left_coord() + field_dimensions.width() / 2) {
            x = field_dimensions.field_x_left_coord() + field_dimensions.width() / 2 -
                BORDER_BUFFER;
        }
    } else if (robot_id_ == 2) {
        // Assign right
        if (x < field_dimensions.field_x_right_coord() - field_dimensions.width() / 2) {
            x = field_dimensions.field_x_right_coord() - field_dimensions.width() / 2 +
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

double Seeking::eval_point(rj_geometry::Point ball_pos, rj_geometry::Point current_point,
                           const WorldState* world_state) {
    // Determines 'how good' a point is
    // A higher value is a worse point

    // Does not go into the goalie boxes
    rj_geometry::Rect goal_box{rj_geometry::Point{1, 8}, rj_geometry::Point{-1, 9}};
    if (goal_box.contains_point(current_point)) {
        return 10000000;
    }

    // Line of Sight Heuristic
    double max = 0;
    double curr_dp;
    for (auto robot : world_state->their_robots) {
        curr_dp = (current_point).norm().dot((robot.pose.position() - ball_pos).norm());
        curr_dp *= curr_dp;
        if (curr_dp > max) {
            max = curr_dp;
        }
    }

    // Whether the path from ball to the point is blocked
    // Same logic in passing to check if target is open
    rj_geometry::Segment pass_path{ball_pos, current_point};
    double min_robot_dist = 10000;
    float min_path_dist = 10000;
    for (auto bot : world_state->their_robots) {
        rj_geometry::Point opp_pos = bot.pose.position();
        min_robot_dist = std::min(min_robot_dist, current_point.dist_to(opp_pos));
        min_path_dist = std::min(min_path_dist, pass_path.dist_to(opp_pos));
    }

    for (auto bot : world_state->our_robots) {
        rj_geometry::Point ally_pos = bot.pose.position();
        min_robot_dist = std::min(min_robot_dist, current_point.dist_to(ally_pos));
        min_path_dist = std::min(min_path_dist, pass_path.dist_to(ally_pos));
    }

    min_path_dist = 0.1 / min_path_dist;
    min_robot_dist = 0.1 / min_robot_dist;

    // More Line of Sight Heuristics
    for (auto robot : world_state->our_robots) {
        curr_dp = (current_point - ball_pos).norm().dot((robot.pose.position() - ball_pos).norm());
        curr_dp *= curr_dp;
        if (curr_dp > max) {
            max = curr_dp;
        }
    }

    // Additional heuristics for calculating optimal point
    double ball_proximity_loss = (current_point - ball_pos).mag() * .002;
    double goal_distance_loss = (9.0 - current_point.y()) * .008;

    // Final evaluation
    return max + ball_proximity_loss + goal_distance_loss + min_path_dist + min_robot_dist;
}

}  // namespace strategy