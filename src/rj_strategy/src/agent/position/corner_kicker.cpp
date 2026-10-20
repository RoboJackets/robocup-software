#include "rj_strategy/agent/position/corner_kicker.hpp"

// dont forget the two includes

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

    std::optinal<RobotIntent> CornerKicker::derived_get_task(RobotIntent intent) {
        if (!last_world_state_) {
            return std::nullopt;
        }

        target_ = calculate_best_pass_target();
        latest_state_ = update_state();
        return state_to_task(intent);
    }

    CornerKicker::State CornerKicker::update_state() {}

    std::optional<RobotIntent> CornerKicker::state_to_task(RobotIntent intent) {}

    rj_geometry::Point CornerKicker::calculate_best_pass_target() const {}
}