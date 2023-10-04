#include "goalie.hpp"

#include <spdlog/spdlog.h>

namespace strategy {

Goalie::Goalie(int r_id) : Position(r_id) { position_name_ = "Goalie"; }

std::optional<RobotIntent> Goalie::derived_get_task(RobotIntent intent) {
    latest_state_ = update_state();
    return state_to_task(intent);
}

Goalie::State Goalie::update_state() {
    // if a shot is coming, override all and go block it
    WorldState* world_state = last_state_world;

    // if no ball found, stop and return to box immediately
    if (!world_state->ball.visible) {
        return BALL_NOT_FOUND;
    }

    // if a shot is coming, override all and go block it
    if (shot_on_goal_detected(world_state)) {
        return BLOCKING;
    }

    // if the ball is in the goalie box, clear it
    bool ball_is_slow = world_state->ball.velocity.mag() < 0.5;  // m/s

    rj_geometry::Point ball_pt = world_state->ball.position;

    bool ball_in_box = this->field_dimensions_.our_defense_area().contains_point(ball_pt);
    if (ball_is_slow && ball_in_box) {
        if (latest_state_ != CLEARING) {
            return PREPARING_SHOT;
        } else if (latest_state_ == PREPARING_SHOT && check_is_done()) {
            // when PREPARING_SHOT is done, CLEAR
            return CLEARING;
        }
    }

    // when line kick fails but ball leaves box, don't chase it
    if (!ball_in_box) {
        return IDLING;
    }

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    double distance_to_ball = robot_position.dist_to(ball_pt);
    if (latest_state_ == PASSING) {
        if (!ball_in_box) {
            return IDLING;
        }
    } else if (latest_state_ == RECEIVING) {
        if (distance_to_ball < 0.1) {
            return IDLING;
        }
    }

    return latest_state_;
}

std::optional<RobotIntent> Goalie::state_to_task(RobotIntent intent) {
    if (latest_state_ == BLOCKING) {
        planning::LinearMotionInstant target{rj_geometry::Point{0.0, 0.1}};
        auto intercept_cmd = planning::MotionCommand{"intercept", target};
        intent.motion_command = intercept_cmd;
        return intent;
    } else if (latest_state_ == IDLING) {
        auto goalie_idle_cmd = planning::MotionCommand{"goalie_idle"};
        intent.motion_command = goalie_idle_cmd;
        return intent;
    } else if (latest_state_ == PREPARING_SHOT) {
        // pivot around ball...
        auto ball_pt = last_state_world->ball.position;

        // ...to face their goal
        planning::LinearMotionInstant target_instant{clear_point_};

        auto pivot_cmd = planning::MotionCommand{"pivot"};
        pivot_cmd.target = target_instant;
        pivot_cmd.pivot_point = ball_pt;
        intent.motion_command = pivot_cmd;
        intent.dribbler_speed = 255.0;
        return intent;
    } else if (latest_state_ == CLEARING) {
        planning::LinearMotionInstant target{clear_point_};
        auto line_kick_cmd = planning::MotionCommand{"line_kick", target};
        intent.motion_command = line_kick_cmd;

        // note: the way this is set up makes it impossible to
        // shoot on time without breakbeam
        // TODO(Kevin): make intent hold a manip msg instead? to be cleaner?
        intent.shoot_mode = RobotIntent::ShootMode::CHIP;
        intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
        intent.kick_speed = 4.0;
        intent.dribbler_speed = 255.0;
        intent.is_active = true;

        return intent;
    } else if (latest_state_ == BALL_NOT_FOUND) {
        // TODO: make point dependent on team
        rj_geometry::Point target_pt = this->field_dimensions_.our_defense_area().center();
        rj_geometry::Point target_vel{0.0, 0.0};

        planning::PathTargetFaceOption face_option = planning::FaceTarget{};

        // ball not found
        bool ignore_ball = true;

        planning::LinearMotionInstant target{target_pt, target_vel};
        intent.motion_command =
            planning::MotionCommand{"path_target", target, face_option, ignore_ball};
        return intent;
    } else if (latest_state_ == RECEIVING) {
        // TODO(https://app.clickup.com/t/8677rrgjn): Convert RECEIVING state into role_interface
        // intercept the bal
        rj_geometry::Point current_position =
            last_state_world->get_robot(true, robot_id_).pose.position();
        planning::LinearMotionInstant target{current_position};
        auto receive_intercept_cmd = planning::MotionCommand{"intercept", target};
        intent.motion_command = receive_intercept_cmd;
        return intent;
    } else if (latest_state_ == PASSING) {
        // TODO(https://app.clickup.com/t/8677rrgjn): Convert PASSING state into role_interface
        // attempt to pass the ball to the target robot
        rj_geometry::Point target_robot_pos =
            last_state_world->get_robot(true, target_robot_id).pose.position();
        planning::LinearMotionInstant target{target_robot_pos};
        auto pass_kick_cmd = planning::MotionCommand{"line_kick", target};
        intent.motion_command = pass_kick_cmd;
        intent.shoot_mode = RobotIntent::ShootMode::KICK;
        intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
        // TODO: Adjust the kick speed based on distance
        intent.kick_speed = 4.0;
        intent.is_active = true;
        return intent;
    }

    // should be impossible to reach, but this is equivalent to
    // sending an empty MotionCommand
    return std::nullopt;
}

bool Goalie::shot_on_goal_detected(WorldState* world_state) {
    rj_geometry::Point ball_pos = world_state->ball.position;
    rj_geometry::Point ball_vel = world_state->ball.velocity;

    // find x-coord that the ball would cross on the goal line to figure out if
    // shot is on target ((0, 0) is our goal, +y points out of goal)
    //
    // assumes ball vel will remain constant
    // TODO(Kevin): account for acceleration?
    if (ball_vel.y() == 0) {
        return false;
    }
    double time_to_cross = std::abs(ball_pos.y() / ball_vel.y());
    double cross_x = ball_pos.x() + ball_vel.x() * time_to_cross;

    bool shot_on_target = std::abs(cross_x) < this->field_dimensions_.goal_width() / 2.0;

    bool ball_is_fast = ball_vel.mag() > 1.0;
    return ball_is_fast && shot_on_target;
}

void Goalie::derived_acknowledge_pass() { latest_state_ = FACING; }

void Goalie::derived_pass_ball() { latest_state_ = PASSING; }

void Goalie::derived_acknowledge_ball_in_transit() { latest_state_ = RECEIVING; }

}  // namespace strategy
