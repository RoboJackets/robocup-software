#pragma once

#include "rj_strategy/agent/position.hpp"

namespace strategy {

class Runner : public Position {
public:
    Runner(int robot_id);
    Runner(Position&& other);  // Converted to a Move Constructor
    ~Runner() override = default;

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    std::string get_current_state() override;

private:
    enum class State { SIDE_1, SIDE_2, SIDE_3, SIDE_4 };

    State current_state_ = State::SIDE_1;

    void update_state();
    std::optional<RobotIntent> state_to_task(RobotIntent intent);
};

}  // namespace strategy