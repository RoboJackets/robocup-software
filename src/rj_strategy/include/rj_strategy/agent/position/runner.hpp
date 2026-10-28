#pragma once

#include <optional>
#include <string>

#include <rj_geometry/point.hpp>

#include "rj_strategy/agent/position.hpp"

namespace strategy {

class Runner : public Position {
public:
    Runner(int r_id);
    Runner(Position&& other);
    ~Runner() override = default;

    std::string get_current_state() override;

    std::string get_state_name() const override {
        return std::string{state_to_name(current_state_)};
    }

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State { SIDE_1, SIDE_2, SIDE_3, SIDE_4 };

    static constexpr const char* state_to_name(State s) {
        switch (s) {
            case SIDE_1:
                return "SIDE_1";
            case SIDE_2:
                return "SIDE_2";
            case SIDE_3:
                return "SIDE_3";
            case SIDE_4:
                return "SIDE_4";
        }
        return "UNKNOWN";
    }

    State next_state();
    std::optional<RobotIntent> state_to_task(RobotIntent intent);
    rj_geometry::Point corner(State s) const;

    State current_state_ = State::SIDE_1;

    static constexpr double kHalfWidth{1.5};
    static constexpr double kHalfHeight{1.0};

    static constexpr double kArrivedRadius{0.2};
};

}  // namespace strategy
