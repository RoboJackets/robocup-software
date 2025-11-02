#pragma once

#include <rclcpp/rclcpp.hpp>

#include "rj_strategy/agent/position.hpp"

namespace strategy {
class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;
    Runner(const Position& other);

    std::string get_current_state() override;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State { RUNNING };

    State update_state();
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    State current_state_ = State::RUNNING;
    int current_vertex_index_ = 0;
    static constexpr int kNumVertices = 4;
};
}  // namespace strategy
