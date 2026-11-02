#pragma once

#include <optional>
#include <string>

#include "rj_strategy/agent/position.hpp"

namespace strategy {

/**
 * Runner: drives laps around a rectangle, one side at a time.
 *
 * Each state is one side of the rectangle (the robot is driving toward the
 * state's corner). When the move finishes, we advance to the next side.
 */
class Runner : public Position {
public:
    Runner(int r_id);
    Runner(const Position& other);
    ~Runner() override = default;

    std::string get_current_state() override;

    // Required by Position; Runner doesn't communicate, so these do nothing.
    void receive_communication_response(communication::AgentPosResponseWrapper response) override;
    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;

private:
    enum State { SIDE_1, SIDE_2, SIDE_3, SIDE_4 };

    State current_state_ = SIDE_1;

    // Rectangle corners (meters, same coordinates as the simulator cursor).
    // The robot drives to these in order: SIDE_1 -> corner 0, SIDE_2 -> corner 1, ...
    // Adjust these to fit your field.
    static constexpr double kCorners[4][2] = {
        {-1.5, 1.0},
        {1.5, 1.0},
        {1.5, 2.5},
        {-1.5, 2.5},
    };

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    State next_state();
    std::optional<RobotIntent> state_to_task(RobotIntent intent);
};

}  // namespace strategy