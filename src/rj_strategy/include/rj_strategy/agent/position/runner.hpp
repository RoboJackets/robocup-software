#pragma once

#include <string>

#include "rj_strategy/agent/position.hpp"

namespace strategy {

/**
 * Tutorial position. The robot runs a rectangle inside the field, one side
 * per state, and starts the rectangle over after the fourth side.
 */
class Runner : public Position {
public:
    Runner(int r_id);
    // Required so RobotFactoryPosition can swap into this position.
    Runner(Position&& other);
    ~Runner() override = default;

    std::string get_current_state() override;
    std::string get_state_name() const override;

private:
    // One state per side. The value is also the index of the corner that side ends on.
    enum State {
        SIDE_ONE = 0,
        SIDE_TWO,
        SIDE_THREE,
        SIDE_FOUR,
    };

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    // Advance to the next side once the planner reports this side is finished.
    State next_state();

    // Drive in a straight line to the corner at the end of the current side.
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    // Four corners, inset so the lap stays on the field and out of both defense areas.
    rj_geometry::Point corner_for(State state) const;

    State current_state_ = SIDE_ONE;
};

}  // namespace strategy
