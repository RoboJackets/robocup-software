#include "rj_strategy/agent/position/corner_kicker.hpp"


namespace strategy {
    CornerKicker::CornerKicker(int r_id) : Position(r_id) {
        latest_state_ = APPROACHING;
    }

    CornerKicker::CornerKicker(Position&& other) : Position{std::move(other)} {
        latest_state_ = APPROACHING;
    }

    // functions from header
    void CornerKicker::derived_acknowledge_pass() {}
    void CornerKicker::derived_pass_ball() {}
    void CornerKicker::derived_acknowledge_ball_in_transit() {}

    std::string CornerKicker::get_current_state() {
        switch (latest_state_) {
            case State::APPROACHING:
                return"CornerKicker: APPROACHING";
            case State::LINE_UP:
                return"CornerKicker: LINE_UP";
            case State::KICKING:
                return"CornerKicker: KICKING";
        }
        return "CornerKicker: UNKNOWN";
    }

    std::optional<RobotIntent> CornerKicker::derived_get_task(RobotIntent intent) {
        if (latest_state_ == APPROACHING) {
            target_ = calculate_best_pass_target(); // put in here so doesnt recompute every tick
        }


        latest_state_ = update_state();
        return state_to_task(intent);
    }

    CornerKicker::State CornerKicker::update_state() {
        // make switch case for every state ERIC implemented
        switch (latest_state_) {
            case APPROACHING: {
                // reached spot behind ball and will align
                if (check_is_done()) {
                    return LINE_UP;
                }
                break;
            }
            case LINE_UP: {
                // aligned or touching ball
                if (check_is_done() || distance_to_ball() < kOwnBallRadius) {
                    return KICKING;
                }
                break;
            }
            case KICKING:{
                // kick finished
                if (check_is_done()) {
                    return APPROACHING;
                }
                break;
            }
        }
        return latest_state_;
    }
    std::optional<RobotIntent> CornerKicker::state_to_task(RobotIntent intent) {
        rj_geometry::Point ball = last_world_state_->ball.position;

        // unit vector pointing from ball to preferred kick direction
        rj_geometry::Point kick_dir = (target_ - ball).normalized(1.0);

        switch(latest_state_) {
            case APPROACHING: {
                // go to a spot kApproachDist behind the ball, on the opposite side from target_
                rj_geometry::Point stage_pt = ball - kick_dir * kApproachDist;
                planning::LinearMotionInstant goal{stage_pt, rj_geometry::Point{0.0, 0.0}};
                intent.motion_command =
                    planning::MotionCommand{"path_target", goal, planning::FaceBall{}};
                return intent;
            }
            case LINE_UP: {
                rj_geometry::Point line_pt = ball - kick_dir * kOwnBallRadius;
                planning::PathTargetFaceOption face_option{planning::FaceBall{}};
                bool ignore_ball{false};  // keep avoiding the ball until we're lined up
                planning::LinearMotionInstant goal{line_pt, rj_geometry::Point{0.0, 0.0}};
                intent.motion_command =
                    planning::MotionCommand{"path_target", goal, face_option, ignore_ball};
                return intent;
            }
            case KICKING: {
                intent.motion_command = planning::MotionCommand{"line_kick", planning::LinearMotionInstant{target_}};
                intent.shoot_mode = RobotIntent::ShootMode::KICK;
                intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
                intent.kick_speed = 4.0;
                return intent;
            }
        }
        return intent;
    }

    rj_geometry::Point CornerKicker::calculate_best_pass_target() const {
        // get goal position from field dimensions
        rj_geometry::Point goal = field_dimensions_.their_goal_loc(); // opponent goal center

        std::optional<rj_geometry::Point> best;
        double best_dist = std::numeric_limits<double>::max();
        
        const auto& allies = last_world_state_->our_robots; 
        // LOOP THROUGH ALLIED ROBOTS
        for (size_t i = 0; i < allies.size(); i++) {
            if (static_cast<int>(i) == robot_id_) continue;       // skip self
            if (!allies[i].visible) continue;                     // not on field

            rj_geometry::Point pos = allies[i].pose.position();

            // nudge toward goal so reciever can step into ball, 
            // includes cases when robot is within 0.3 m of goal
            auto to_goal = goal - pos;
            rj_geometry::Point candidate = pos + to_goal.normalized(std::min(0.3, to_goal.mag()));

            // classic 1332 graph lines
            double d = candidate.dist_to(goal);
            if (d < best_dist) {
                best_dist = d;
                best = candidate;
            }
        }

        if (best) return *best;

        // DEFAULT if there are no robots
        return goal;
    }
}