#include "solo_offense.hpp"

namespace strategy {

SoloOffense::SoloOffense(const Position& other) : Position{other} {
    position_name_ = "SoloOffense";
}

SoloOffense::SoloOffense(int r_id) : Position{r_id, "SoloOffense"} {}

std::optional<RobotIntent> SoloOffense::derived_get_task(RobotIntent intent) {
    // Get next state, and if different, reset clock
    State new_state = next_state();
    // if (new_state != current_state_) {
    // }
    // SPDLOG_INFO("New State: {}", std::to_string(static_cast<int>(new_state)));
    current_state_ = new_state;

    // Calculate task based on state
    return state_to_task(intent);
}

std::string SoloOffense::get_current_state() {
    return std::string{"Solo Offense"} + std::to_string(static_cast<int>(current_state_));
}

SoloOffense::State SoloOffense::next_state() {

    switch (current_state_) {
        case START: {
            if (check_is_done()) {
                return POINT_US;
            }
            return START;
        }
        case POINT_US: {
            if (check_is_done()) {
                return POINT_THEM;
            }
            return POINT_US;
        }
        case POINT_THEM: {
            if (check_is_done()) {
                return POINT_US;
            }
            return POINT_THEM;
        }
    }
    return current_state_;
}

std::optional<RobotIntent> SoloOffense::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case START: {
            rj_geometry::Point target{0.0, 4.5};
            auto mark_cmd = planning::MotionCommand{
                "path_target", planning::LinearMotionInstant{target}, planning::FaceBall{}, true};
            intent.motion_command = mark_cmd;
            return intent;
        }
        case POINT_US: {
            planning::LinearMotionInstant target{field_dimensions_.our_goal_loc()};
            auto pivot_cmd =
                planning::MotionCommand{"rotate", target, planning::FaceTarget{}, false};
            intent.motion_command = pivot_cmd;
            return intent;
        }
        case POINT_THEM: { 
            planning::LinearMotionInstant target{field_dimensions_.their_goal_loc()};
            auto pivot_cmd =
                planning::MotionCommand{"rotate", target, planning::FaceTarget{}, false};
            intent.motion_command = pivot_cmd;
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
