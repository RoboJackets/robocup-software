#include "free_kicker.hpp"

namespace strategy {

FreeKicker::FreeKicker(int r_id) : Position(r_id, "FreeKicker") {}

FreeKicker::FreeKicker(const Position& other) : Position{other} {
    position_name_ = "FreeKicker";
    cached_start_time_ = RJ::now();
}

std::optional<RobotIntent> FreeKicker::derived_get_task(RobotIntent intent) {
    current_state_ = next_state();

    return state_to_task(intent);
}

FreeKicker::State FreeKicker::next_state() {
    switch (current_state_) {
        case APPROACH: {
            if ((check_is_done() && current_play_state_.is_ready()) ||
                (RJ::now() - *cached_start_time_ > RJ::Seconds(7.5))) {
                cached_start_time_.reset();
                return KICK;
            }
            break;
        }
        case KICK: {
            if (check_is_done()) {
                cached_start_time_ = RJ::now();
                return APPROACH;
            }
            break;
        }
    };
    return current_state_;
}

std::optional<RobotIntent> FreeKicker::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case APPROACH: {
            double y_pos = field_dimensions_.their_goal_loc().y();

            rj_geometry::Point target_pt{last_world_state_->ball.position.x(), y_pos};
            rj_geometry::Point target_vel{0.0, 0.0};
            // Face ball
            planning::PathTargetFaceOption face_option{planning::FaceTarget{}};
            // Avoid ball
            bool ignore_ball{false};

            // Create Motion Command
            planning::LinearMotionInstant goal{target_pt, target_vel};
            auto motion_command = planning::MotionCommand{
                "line_pivot", goal, face_option, ignore_ball, last_world_state_->ball.position};

            motion_command.pivot_radius = 0.5;

            for (auto border : field_dimensions_.field_borders()) {
                if (border.dist_to(last_world_state_->ball.position) <
                    (motion_command.pivot_radius * 1.1)) {
                    motion_command.pivot_radius = 0.15;
                    break;
                }
            }

            intent.motion_command = motion_command;

            return intent;
        }
        case KICK: {
            rj_geometry::Point goal_corner{this->field_dimensions_.their_goal_loc().x() +
                                               0.5 * this->field_dimensions_.goal_width(),
                                           this->field_dimensions_.their_goal_loc().y()};

            planning::LinearMotionInstant target{goal_corner};
            auto line_kick_cmd = planning::MotionCommand{"line_kick", target};
            intent.motion_command = line_kick_cmd;

            // note: the way this is set up makes it impossible to
            // shoot on time without breakbeam
            intent.shoot_mode = RobotIntent::ShootMode::KICK;
            intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
            intent.kick_speed = 4.0;
            intent.is_active = true;

            return intent;
        }
    };
    return nullopt;
}

std::string FreeKicker::get_current_state() { return "FreeKicker"; }

void FreeKicker::derived_acknowledge_pass() {}

void FreeKicker::derived_pass_ball() {}

void FreeKicker::derived_acknowledge_ball_in_transit() {}

}  // namespace strategy
