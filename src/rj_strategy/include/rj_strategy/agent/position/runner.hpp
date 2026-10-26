#pragma once

#include <optional>
#include <string>
#include <string_view>

#include <rj_common/planning/instant.hpp>
#include <rj_geometry/point.hpp>

#include "rj_strategy/agent/position.hpp"

namespace strategy {
class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;
    Runner(Position&& other);

    std::string get_current_state() override;

    std::string get_state_name() const override {
        return std::string(state_to_name(current_state_));
    }

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State {
        SIDE_ONE,
        SIDE_TWO,
        SIDE_THREE,
        SIDE_FOUR,
    };

    State next_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    static constexpr std::string_view state_to_name(State s) {
        switch (s) {
            case SIDE_ONE:
                return "SIDE_ONE";
            case SIDE_TWO:
                return "SIDE_TWO";
            case SIDE_THREE:
                return "SIDE_THREE";
            case SIDE_FOUR:
                return "SIDE_FOUR";
        }
        return "UNKNOWN";
    }

    State current_state_ = SIDE_ONE;
};
}