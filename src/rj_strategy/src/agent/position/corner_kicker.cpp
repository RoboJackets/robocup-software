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
        if (!last_world_state_) {
            return std::nullopt;
        }

        target_ = calculate_best_pass_target();
        latest_state_ = update_state();
        return state_to_task(intent);
    }

    CornerKicker::State CornerKicker::update_state() {
        return latest_state_;
    }
    std::optional<RobotIntent> CornerKicker::state_to_task(RobotIntent intent) {
        return intent;
    }

    rj_geometry::Point CornerKicker::calculate_best_pass_target() const {
        // get goal position from field dimensions
        rj_geometry::Point goal = field_dimensions_.their_goal_loc(); // opponent goal center

        std::optional<rj_geometry::Point> best;
        double best_dist = std::numeric_limits<double>::max();
        
        const auto& allies = last_world_state_->our_robots; // ERIC:::: verify name???
        // LOOP THROUGH ALLIED ROBOTS
        for (size_t i = 0; i < allies.size(); i++) {
            if (static_cast<int>(i) == robot_id_) continue;       // if id is me, skip
            if (!allies[i].visible) continue                    // verify flag name??

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