#pragma once

#include "rj_strategy/agent/position.hpp"

namespace strategy {

class Rotate : public Position {
public:
    // Define the 4 target states for the rotation
    enum class RotateState {
        DEG_0 = 0,
        DEG_90 = 1,
        DEG_180 = 2,
        DEG_270 = 3
    };

    Rotate(int r_id);
    Rotate(int r_id, bool clockwise, bool sweep_mode = true);
    Rotate(Position&& other);
    ~Rotate() override = default;

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

private:
    bool clockwise_ = true;
    bool sweep_mode_ = true;
    bool sweeping_up_ = true;
    RotateState state_ = RotateState::DEG_0;
};

}  // namespace strategy
