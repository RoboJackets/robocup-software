#pragma once

#include <optional>
#include <string>

#include <rj_geometry/point.hpp>

#include "rj_strategy/agent/position.hpp"

namespace strategy {

/**
 * The Runner laps the field: it drives to each corner of a rectangle (inside
 * the sidelines, past each defense area) counterclockwise, then starts over.
 */
class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;
    Runner(Position&& other);

    std::string get_current_state() override;
    std::string get_state_name() const override;

private:
    enum State { DEFAULT, SIDE_ONE, SIDE_TWO, SIDE_THREE, SIDE_FOUR };

    State current_state_ = DEFAULT;

    /**
     * @brief Overriden from Position. Calls next_state and then state_to_task on each tick.
     */
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    State next_state();
    bool reached_current_target() const;
    std::optional<RobotIntent> state_to_task(RobotIntent intent);
    rj_geometry::Point corner_for(State state) const;
};

}  // namespace strategy