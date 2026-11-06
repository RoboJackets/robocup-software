#pragma once
#include "rj_strategy/agent/position.hpp"

namespace strategy {

class Runner : public Position {


public:
    Runner(int r_id);
    std::string get_current_state() override;

private:

    enum State {
        DEFAULT,
        TOP,
        RIGHT,
        BOTTOM,
        LEFT
    };

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    State next_state();
    std::optional<RobotIntent> state_to_task(RobotIntent intent);
    State current_state_ = State::DEFAULT;
};

}
