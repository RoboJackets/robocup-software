#include "rj_strategy/agent/position/solo_offense.hpp"

namespace strategy {

SoloOffense::SoloOffense(Position&& other) : Position{std::move(other)} {
    position_name_ = "SoloOffense";
}

SoloOffense::SoloOffense(int r_id) : Position{r_id, "SoloOffense"} {}

std::string SoloOffense::get_current_state() {
    return std::string{"Solo Offense ("} + std::to_string(robot_id_) + std::string{") - "} +
           std::string{state_to_name(current_state_)};
}

std::optional<RobotIntent> SoloOffense::derived_get_task(RobotIntent intent) {
    State new_state = next_state();
    current_state_ = new_state;
    return state_to_task(intent);
}

SoloOffense::State SoloOffense::next_state() {
    RobotState& me = last_world_state_->get_robot(true, robot_id_);

    if (!ball_in_play_area(last_world_state_, field_dimensions_)) {
        return IDLE;
    }  // High-level conditional: SoloOffense does not think when the ball is not playable
    if (we_have_ball(last_world_state_, 2 * kRobotRadius) &&
        !robot_has_ball(last_world_state_, me, 2 * kRobotRadius)) {
        return IDLE;
    }  // High-level conditional: SoloOffense does not think when teammates are handling the ball
    if (they_have_ball(last_world_state_, 2 * kRobotRadius)) {
        return MARKER;
    }  // High-level conditional: SoloOffense is bullyable; if the opponents have the ball, they
       // give up shooting and camp the ball
    // ^^ that's particularly bad strategy, it's legal for opponents to just roll up on us, but
    // whatever, this is a test Position

    switch (current_state_) {
        case IDLE: {
            // When thinking, SoloOffense will immediately leave IDLE.
            kick_target_ = planning::LinearMotionInstant{
                calculate_best_shot(last_world_state_, field_dimensions_)};
            return kick_strategy_;
        }
        case MARKER: {
            // If the opponent has lost possession (see above), SoloOffense will attempt a collect.
            kick_target_ = planning::LinearMotionInstant{
                calculate_best_shot(last_world_state_, field_dimensions_)};
            return kick_strategy_;
        }
        case TO_BALL: {
            // Once we've reached the ball, line kick it.
            if (check_is_done()) {
                return KICK;
            } else {
                return TO_BALL;
            }
            // the high-level conditionals catch normal game cases, but suppose a HALT interrupts a
            // TO_BALL state, would it resume in TO_BALL?
        }
        case KICK: {
            // If a kick is successful, restart the logic tree.
            if (check_is_done()) {
                return IDLE;
            } else {
                return KICK;
            }
        }
        default: {
            return current_state_;  // unreachable, but compiler wants it
        }
    }
}

std::optional<RobotIntent> SoloOffense::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case IDLE: {
            return intent;  // TODO: how to notate an idling intent?
        }
        case MARKER: {
            // We want to be 5 radii from the ball toward the goal; blocking shots! baskingball :)
            rj_geometry::Point ball_pos = last_world_state_->ball.position;
            rj_geometry::Point defending_pos = field_dimensions_.our_goal_loc();
            rj_geometry::Point offset_vector =
                (defending_pos - ball_pos).normalized(kRobotRadius * 5);
            rj_geometry::Point target_pos = ball_pos + offset_vector;

            auto mark_cmd =
                planning::MotionCommand{"path_target", planning::LinearMotionInstant{target_pos},
                                        planning::FaceBall{}, true};
            intent.motion_command = mark_cmd;
            return intent;
        }
        case TO_BALL: {
            // Drive up to the ball (no dribbler available, so just approach it).
            auto approach_ball_cmd = planning::MotionCommand{
                "path_target", planning::LinearMotionInstant{last_world_state_->ball.position},
                planning::FaceBall{}};
            intent.motion_command = approach_ball_cmd;
            return intent;
        }
        case KICK: {
            // Drive behind the ball, then shoot with a run-up.
            auto kick_cmd =
                planning::MotionCommand{"line_kick", kick_target_, planning::FaceTarget{}, true};
            intent.motion_command = kick_cmd;
            intent.shoot_mode = RobotIntent::ShootMode::KICK;
            intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
            intent.kick_speed = max_kick_speed();
            return intent;
        }
    }
    return intent;
}

}  // namespace strategy
