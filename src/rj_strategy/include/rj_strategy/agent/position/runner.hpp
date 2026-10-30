#pragma once

#include "rj_strategy/agent/position.hpp"
#include "rj_strategy/agent/position_utils.hpp"
#include <rj_common/planning/motion_command.hpp>
#include <rj_geometry/point.hpp>


namespace strategy {

class Runner : public Position {

public:
    Runner(int r_id);
    ~Runner() override = default;
    Runner(Position&& other);
    
    std::string get_current_state() override;

private:
    enum State { SIDE1, SIDE2, SIDE3, SIDE4};

    State next_state();

    State current_state_ = State::SIDE1;

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    std::optional<RobotIntent> state_to_task(RobotIntent intent);
};

}