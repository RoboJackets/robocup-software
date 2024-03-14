#include "seeker.hpp"

namespace strategy {

Seeker::Seeker(int robot_id) { robot_id_ = robot_id; }

std::optional<RobotIntent> Seeker::get_task(RobotIntent intent, const WorldState* last_world_state,
                                            FieldDimensions field_dimensions) {
    // Determine target position for seeking
    rj_geometry::Point current_loc = last_world_state->get_robot(true, robot_id_).pose.position();

    if (!target_valid_) {
        target_pt_ = get_open_point(last_world_state, current_loc, field_dimensions);
        target_valid_ = true;
    }

    planning::PathTargetFaceOption face_option = planning::FaceBall{};
    bool ignore_ball = false;
    planning::LinearMotionInstant goal{target_pt_, rj_geometry::Point{0.0, 0.0}};
    intent.motion_command = planning::MotionCommand{"path_target", goal, face_option, ignore_ball};

    return intent;
}

void Seeker::reset_target() { target_valid_ = false; }

rj_geometry::Point Seeker::get_target_point() { return target_pt_; }

void Seeker::set_seeker_points(const std::unordered_map<int, rj_geometry::Point>& seeker_points) {
    seeker_points_ = seeker_points;
}

rj_geometry::Point Seeker::get_open_point(const WorldState* world_state,
                                          rj_geometry::Point current_position,
                                          const FieldDimensions& field_dimensions) const {
    return Seeker::calculate_open_point(3.0, .2, current_position, world_state, field_dimensions);
}

rj_geometry::Point Seeker::calculate_open_point(double current_prec, double min_prec,
                                                rj_geometry::Point current_point,
                                                const WorldState* world_state,
                                                const FieldDimensions& field_dimensions) const {
    while (current_prec > min_prec) {
        rj_geometry::Point ball_pos = world_state->ball.position;
        rj_geometry::Point min = current_point;
        double min_val = eval_point(ball_pos, current_point, world_state, field_dimensions);
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
            curr_val = eval_point(ball_pos, point, world_state, field_dimensions);
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

rj_geometry::Point Seeker::correct_point(rj_geometry::Point p,
                                         const FieldDimensions& field_dimensions) const {
    double border_buffer = .2;
    double x = p.x();
    double y = p.y();

    // X Border
    if (p.x() > field_dimensions.field_x_right_coord() - border_buffer) {
        x = field_dimensions.field_x_right_coord() - border_buffer;
    } else if (p.x() < field_dimensions.field_x_left_coord() + border_buffer) {
        x = field_dimensions.field_x_left_coord() + border_buffer;
    }

    // Y Border
    if (p.y() > field_dimensions.their_goal_loc().y() - border_buffer) {
        y = field_dimensions.their_goal_loc().y() - border_buffer;
    } else if (p.y() < field_dimensions.our_goal_loc().y() + border_buffer) {
        y = field_dimensions.our_goal_loc().y() + border_buffer;
    }

    // Goalie Boxes
    if ((y < 1.2 || y > 7.8) && fabs(x) < 1.2) {
        if (y > 4.5) {
            y = 8.0 - border_buffer;
        } else {
            y = 1.0 + border_buffer;
        }

        if (x > .5) {
            x = 1.0 + border_buffer;
        } else {
            x = -1.0 - border_buffer;
        }
    }

    return rj_geometry::Point(x, y);
}

double Seeker::eval_point(rj_geometry::Point ball_pos, rj_geometry::Point current_point,
                          const WorldState* world_state,
                          const FieldDimensions& field_dimensions) const {
    // Determines 'how good' a point is
    // A higher value is a worse point

    // Does not go into the goalie boxes
    rj_geometry::Rect goal_box{rj_geometry::Point{1, 8}, rj_geometry::Point{-1, 9}};
    if (goal_box.contains_point(current_point)) {
        return std::numeric_limits<double>::infinity();
    }

    // Line of Sight Heuristic
    double max = 0;
    double curr_dp = 0;
    for (const RobotState& robot : world_state->their_robots) {
        curr_dp = (current_point).norm().dot((robot.pose.position() - ball_pos).norm());
        curr_dp *= curr_dp;
        if (curr_dp > max) {
            max = curr_dp;
        }
    }

    // Whether the path from ball to the point is blocked
    // Same logic in passing to check if target is open
    rj_geometry::Segment pass_path{ball_pos, current_point};
    double min_robot_dist = std::numeric_limits<double>::infinity();
    float min_path_dist = std::numeric_limits<double>::infinity();
    
    for (const RobotState& robot : world_state->their_robots) {
        rj_geometry::Point opp_pos = robot.pose.position();
        auto robot_dist = current_point.dist_to(opp_pos);
        min_robot_dist = std::min(min_robot_dist, robot_dist);
        auto path_dist = pass_path.dist_to(opp_pos);
        min_path_dist = std::min(min_path_dist, path_dist);
    }

    for (const RobotState& robot : world_state->our_robots) {
        rj_geometry::Point ally_pos = robot.pose.position();
        auto robot_dist = current_point.dist_to(ally_pos);
        // if dist is 0, then bot must be seeker (self ) robot, so should ignore
        if (robot_dist == 0) {
            continue;
        }
        min_robot_dist = std::min(min_robot_dist, robot_dist);
        auto path_dist = pass_path.dist_to(ally_pos);
        min_path_dist = std::min(min_path_dist, path_dist);
    }

    min_path_dist = 0.1f / min_path_dist;
    min_robot_dist = 0.1 / min_robot_dist;

    // More Line of Sight Heuristics
    for (const RobotState& robot : world_state->our_robots) {
        curr_dp = (current_point - ball_pos).norm().dot((robot.pose.position() - ball_pos).norm());
        curr_dp *= curr_dp;
        if (curr_dp > max) {
            max = curr_dp;
        }
    }

    // Additional heuristics for calculating optimal point
    const double ball_proximity_loss_weight = 0.002;
    const double goal_distance_loss_weight = 0.15;
    double ball_proximity_loss = (current_point - ball_pos).mag() * ball_proximity_loss_weight;
    double goal_distance_loss = (field_dimensions.their_defense_area().miny() - current_point.y()) *
                                goal_distance_loss_weight;

    rj_geometry::Segment ball_goal{ball_pos, field_dimensions.their_goal_loc()};
    double block_shot_dist = ball_goal.dist_to(current_point);
    double block_shot_loss = 0;
    if (block_shot_dist < 1) {
        block_shot_loss = 1;
    }

    // Finding the minimum distance from the target point to the other seeker points (from
    // communication) Heuristic to penalize being close to other seekers (a small minimum distance)s
    double min_seeker_dist = std::numeric_limits<double>::infinity();
    for (const auto& [key, value] : seeker_points_) {
        min_seeker_dist = std::min(min_seeker_dist, current_point.dist_to(value));
    }
    const double seeker_dist_loss = 0.4 * 1 / min_seeker_dist;

    // Final evaluation
    return max + ball_proximity_loss + goal_distance_loss + min_path_dist + min_robot_dist +
           block_shot_loss + seeker_dist_loss;
}

}  // namespace strategy
