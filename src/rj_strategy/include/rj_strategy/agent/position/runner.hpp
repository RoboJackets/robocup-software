#pragma once

#include <array>
#include <optional>
#include <string>
#include <string_view>

#include <rj_geometry/point.hpp>

#include "rj_strategy/agent/position.hpp"
#include "rj_strategy/agent/position_utils.hpp"


//copied from offense.hpp with some modifications to fit the runner position
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
        TOP,
        RIGHT,
        BOTTOM,
        LEFT,
    };

    State next_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    static constexpr std::string_view state_to_name(State s) {
        switch (s) {
            case TOP:
                return "TOP";
            case RIGHT:
                return "RIGHT";
            case BOTTOM:
                return "BOTTOM";
            case LEFT:
                return "LEFT";
        }
        return "UNKNOWN";
    }

    State current_state_ = State::TOP;

    std::array<rj_geometry::Point, 4> corners_;
};
}