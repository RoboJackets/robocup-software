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
            if (check_is_done()) { // TODO: better checking. we should only go to gather step if the ball is in front of us. if TO_BALL has somehow catastrophically failed, it should go back to TO_BALL
                gather_target_ = calculate_gather(); // TODO: need to check if gather_target_ is in bounds
                return GATHER_STEP;
            }
            return TO_BALL;
        }
        case GATHER_STEP: {
            if (check_is_done()) { // TODO: same thing, only go to side step if we have the ball
                juke_target_ = calculate_juke(); // TODO: need to check if juke_target_ is in bounds
                return SIDE_STEP;
            }
            return GATHER_STEP;
        }
        case SIDE_STEP: {
            if (check_is_done()) { // TODO: make a timeout, in case defenders make it annoying to path to our shooting point
                shot_target_ = calculate_best_shot();
                return AIM_AND_SHOOT;
            }
            return SIDE_STEP;
        }
        case AIM_AND_SHOOT: {
            if (check_is_done()) {
                return MARKER;
            }
            return AIM_AND_SHOOT;
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
            rj_geometry::Point robotToBall =
                (last_world_state_->ball.position -
                 last_world_state_->get_robot(true, robot_id_).pose.position());
            double slowDown = 1.0;
            double length = robotToBall.mag() - kRobotRadius * slowDown;
            robotToBall = robotToBall.normalized(length);
            planning::LinearMotionInstant target{
                last_world_state_->get_robot(true, robot_id_).pose.position() + robotToBall};
            auto pivot_cmd = planning::MotionCommand{"collect"};
            intent.motion_command = pivot_cmd;

            return intent;
        }
        case GATHER_STEP: {
            // move a little bit forward to snag the ball in the dribbler, in case collect fumbles it
            // TODO: is there a better motion command to use to just go in a straight line with kicker off and dribbler on?
            auto mark_cmd = planning::MotionCommand{"path_target", planning::LinearMotionInstant{gather_target_}, planning::FaceBall{}, true};
            intent.motion_command = mark_cmd;
            intent.dribbler_mode = RobotIntent::DribblerMode::ON;

            return intent;
        }
        case SIDE_STEP: {
            // rotate to some direction vaguely facing the goal
            auto juke_cmd = planning::MotionCommand{"path_target", planning::LinearMotionInstant{juke_target_}, planning::FaceTarget{}, true};
            intent.motion_command = juke_cmd;
            intent.dribbler_mode = RobotIntent::DribblerMode::ON;

            return intent;
        }
        case AIM_AND_SHOOT: {
            auto pivot_cmd = planning::MotionCommand{"rotate", planning::LinearMotionInstant{shot_target_}, planning::FaceTarget{}, false};
            intent.motion_command = pivot_cmd;
            intent.dribbler_mode = RobotIntent::DribblerMode::ON;
            intent.trigger_mode = RobotIntent::TriggerMode::AT_END;
            intent.kick_speed = 4.0;
            return intent;
        }
    }
    return intent;
}

rj_geometry::Point SoloOffense::calculate_gather() const {
    // move a little bit forward to snag the ball in the dribbler, in case collect fumbles it
    rj_geometry::Pose robot_pose = last_world_state_->get_robot(true, robot_id_).pose;
    return robot_pose.position() + rj_geometry::Point{
        kGatherLength * std::cos(robot_pose.heading()),
        kGatherLength * std::sin(robot_pose.heading())
    };
}

rj_geometry::Point SoloOffense::calculate_juke() const {
    // rotate to some direction vaguely facing the goal
    rj_geometry::Point robo = last_world_state_->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point gol = calculate_best_shot();
    rj_geometry::Point shot_direction = (gol - robo).normalized();
    rj_geometry::Point perp_direction(-shot_direction.y(), shot_direction.x());
    
    double side_step_dist = 0.3; // TODO: random float in (0, 0.9)
    double left_or_right = 1; // TODO: random pick 1 or -1

    rj_geometry::Point forward_offset = shot_direction * 0.1;
    rj_geometry::Point lateral_offset = perp_direction * left_or_right * side_step_dist;

    return robo + forward_offset + lateral_offset;
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
