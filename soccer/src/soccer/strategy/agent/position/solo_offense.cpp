#include "solo_offense.hpp"

namespace strategy {

SoloOffense::SoloOffense(const Position& other) : Position{other} {
    position_name_ = "SoloOffense";
}

SoloOffense::SoloOffense(int r_id) : Position{r_id, "SoloOffense"} {}

std::optional<RobotIntent> SoloOffense::derived_get_task(RobotIntent intent) {
    // Get next state, and if different, reset clock
    State new_state = next_state();
    if (new_state != current_state_) {
        SPDLOG_INFO("New State: {}", std::to_string(static_cast<int>(new_state)));
    }
    current_state_ = new_state;

    // Calculate task based on state
    return state_to_task(intent);
}

std::string SoloOffense::get_current_state() {
    return std::string{"Solo Offense"} + std::to_string(static_cast<int>(current_state_));
}

SoloOffense::State SoloOffense::next_state() {
    // handle transitions between current state
    double closest_dist = std::numeric_limits<double>::infinity();
    auto current_point = last_world_state_->ball.position;

    for (int i = 0; i < 6; i++) {
        RobotState robot = last_world_state_->get_robot(false, i);
        rj_geometry::Point opp_pos = robot.pose.position();
        auto robot_dist = opp_pos.dist_to(current_point);
        if (robot_dist < closest_dist) {
            marking_id_ = i;
            closest_dist = robot_dist;
        }
    }

    // SPDLOG_INFO("Closest dist: {}, i-{}", closest_dist,  marking_id_);

    if (closest_dist < (0.5) || field_dimensions_.their_goal_area().contains_point(current_point) ||
        field_dimensions_.their_defense_area().contains_point(current_point) ||
        !field_dimensions_.field_coordinates().contains_point(current_point)) {
        return MARKER;
    }
    switch (current_state_) {
        case MARKER: {
            return TO_BALL;
        }
        case TO_BALL: {
            // SPDLOG_INFO("{}", (last_world_state_->get_robot(true, robot_id_).pose.position() - current_point).mag());
            // if ((last_world_state_->get_robot(true, robot_id_).pose.position() - current_point).mag() < kRobotDiameter){
            //     return ROTATE;
            // }
            if (check_is_done()) {
                return ROTATE;
            }
            return TO_BALL;
        }
        case ROTATE: {
            if (check_is_done()) {
                counter_ = 0;
                kick_ = true;
                return KICK;
            }
            return ROTATE;
        }
        case KICK: {
            // if (check_is_done() || (last_world_state_->get_robot(true, robot_id_).pose.position() - current_point).mag() > kRobotDiameter * 3) {
            //     return TO_BALL;
            // }
            if (!kick_ || (last_world_state_->get_robot(true, robot_id_).pose.position() - current_point).mag() > kRobotRadius * 5) {
                return TO_BALL;
            }
            return KICK;
        }
    }
    return current_state_;
}

std::optional<RobotIntent> SoloOffense::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case MARKER: {
            auto marker_target_pos =
                last_world_state_->get_robot(false, marking_id_).pose.position();
            auto target =
                marker_target_pos +
                (field_dimensions_.our_goal_loc() - marker_target_pos).normalized(kRobotRadius * 5);
            auto mark_cmd = planning::MotionCommand{
                "path_target", planning::LinearMotionInstant{target}, planning::FaceBall{}, true};
            intent.motion_command = mark_cmd;

            return intent;
        }
        case TO_BALL: {
            rj_geometry::Point robotToBall = (last_world_state_->ball.position - last_world_state_->get_robot(true, robot_id_).pose.position());
            double slowDown = 1.0;
            double length = robotToBall.mag() - kRobotRadius * slowDown;
            robotToBall = robotToBall.normalized(length);
            planning::LinearMotionInstant target{last_world_state_->get_robot(true, robot_id_).pose.position() + robotToBall};
            auto pivot_cmd = planning::MotionCommand{"path_target", target, planning::FaceTarget{}, true};
            intent.motion_command = pivot_cmd;
            intent.dribbler_speed = 255;
            return intent;
        }
        case ROTATE: {
            planning::LinearMotionInstant target{calculate_best_shot()};
            auto pivot_cmd = planning::MotionCommand{"rotate", target, planning::FaceTarget{}, false};
            intent.motion_command = pivot_cmd;
            intent.dribbler_speed = 255;
            return intent;
        }
        case KICK: {
            // double scaleFactor = 0.1;
            // rj_geometry::Point point = (last_world_state_->ball.position - last_world_state_->get_robot(true, robot_id_).pose.position()).normalized(scaleFactor);
            // point += last_world_state_->get_robot(true, robot_id_).pose.position();
            // planning::LinearMotionInstant target{point};
            // SPDLOG_INFO("KICK KICK KICK");
            planning::LinearMotionInstant target{calculate_best_shot()};
            // planning::LinearMotionInstant target{last_world_state_->ball.position};
            auto kick_cmd = planning::MotionCommand{"path_target", target, planning::FaceTarget{}, true};
            intent.motion_command = kick_cmd;
            intent.shoot_mode = RobotIntent::ShootMode::KICK;
            intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
            intent.kick_speed = 4.0;
            counter_++;
            if (counter_ > 15) {
                kick_ = false;
            }

            return intent;
        }
    }
    return intent;
}

rj_geometry::Point SoloOffense::calculate_best_shot() const {
    // Goal location
    rj_geometry::Point their_goal_pos = field_dimensions_.their_goal_loc();
    double goal_width = field_dimensions_.goal_width();  // 1.0 meters

    // Ball location
    rj_geometry::Point ball_position = this->last_world_state_->ball.position;

    rj_geometry::Point best_shot = their_goal_pos;
    double best_distance = -1.0;
    rj_geometry::Point increment(0.05, 0);
    rj_geometry::Point curr_point =
        their_goal_pos - rj_geometry::Point(goal_width / 2.0, 0) + increment;
    for (int i = 0; i < 19; i++) {
        double distance = distance_from_their_robots(ball_position, curr_point);
        if (distance > best_distance) {
            best_distance = distance;
            best_shot = curr_point;
        }
        curr_point = curr_point + increment;
    }
    return best_shot;
}

double SoloOffense::distance_from_their_robots(rj_geometry::Point tail,
                                               rj_geometry::Point head) const {
    rj_geometry::Point vec = head - tail;
    auto& their_robots = this->last_world_state_->their_robots;

    double min_angle = -0.5;
    for (auto enemy : their_robots) {
        rj_geometry::Point enemy_vec = enemy.pose.position() - tail;
        if (enemy_vec.dot(vec) < 0) {
            continue;
        }
        auto projection = (enemy_vec.dot(vec) / vec.dot(vec));
        enemy_vec = enemy_vec - (projection)*vec;
        double distance = enemy_vec.mag();
        if (distance < (kRobotRadius + kBallRadius)) {
            return -1.0;
        }
        double angle = distance / projection;
        if ((min_angle < 0) || (angle < min_angle)) {
            min_angle = angle;
        }
    }
    return min_angle;
}
}  // namespace strategy
