#pragma once

#include <array>
#include <string_view>

#include <rclcpp/rclcpp.hpp>

#include "rj_strategy/agent/position.hpp"

namespace strategy {

class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;
    Runner(Position&& other);

    void derived_acknowledge_pass() override {}
    void derived_pass_ball() override {}
    void derived_acknowledge_ball_in_transit() override {}
    std::string get_current_state() override;

    std::string get_state_name() const override {
        return std::string(state_to_name(current_state_));
    }

private:

    enum State {
        CORNER_1,
        CORNER_2,
        CORNER_3,
        CORNER_4
    };

    State current_state_ = CORNER_1;

    static constexpr std::string_view state_to_name(State s) {
        switch (s) {
            case CORNER_1:
                return "CORNER_1";
            case CORNER_2:
                return "CORNER_2";
            case CORNER_3:
                return "CORNER_3";
            case CORNER_4:
                return "CORNER_4";
        }
    }

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    State next_state();

    // if within the tolerance distance then count as arrived at corner
    static constexpr double cornerDistTolerance{0.15};
    // loop size as a percentage of field length and width
    static constexpr double loopPercentage{0.75};
};

}  // namespace strategy