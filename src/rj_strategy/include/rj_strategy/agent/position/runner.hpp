#pragma once

#include <optional>
#include <string>

#include "rj_strategy/agent/position.hpp"

namespace strategy {

/**
 * Drives a robot in a rectangle
 */
class Runner : public Position {
public:
    explicit Runner(int r_id);
    Runner(Position&& other);

    std::string get_current_state() override;
    std::string get_state_name() const override {
        return std::string(state_to_name(current_state_));
    }

private:
    enum State {
        GO_TO_A,
        GO_TO_B,
        GO_TO_C,
        GO_TO_D,
    };

    static constexpr std::string_view state_to_name(State s) {
        switch (s) {
            case GO_TO_A:
                return "GO_TO_A";
            case GO_TO_B:
                return "GO_TO_B";
            case GO_TO_C:
                return "GO_TO_C";
            case GO_TO_D:
                return "GO_TO_D";
        }
        return "UNKNOWN";
    }

    State current_state_ = State::GO_TO_A;

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    State next_state();
    std::optional<RobotIntent> state_to_task(RobotIntent intent);
};

}  // namespace strategy
