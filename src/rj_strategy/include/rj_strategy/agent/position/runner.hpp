#pragma once

#include <array>
#include <optional>
#include <string>

#include <rj_geometry/point.hpp>

#include "rj_strategy/agent/position.hpp"

namespace strategy {

/**
 * The Runner position drives the robot around the vertices of a rectangle forever.
 * Each state is one side of the rectangle (i.e. "drive to the next corner").
 */
class Runner : public Position {
public:
    explicit Runner(int r_id);
    Runner(Position&& other);
    ~Runner() override = default;

    std::string get_current_state() override;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State {
        SIDE_1,
        SIDE_2,
        SIDE_3,
        SIDE_4,
    };

    /**
     * @return what the state should be right now. Called each tick before state_to_task.
     */
    State next_state();

    /**
     * @return the task to execute. Called each tick AFTER next_state().
     */
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    State current_state_ = State::SIDE_1;

    // Corners of the rectangle; SIDE_i drives to vertices_[i]
    // std::array<rj_geometry::Point, 4> vertices_;

    // True once we've issued the motion command for the current state, so a stale
    // "done" from the previous side can't make us skip ahead.
    bool command_sent_ = false;

    std::array<rj_geometry::Point, 4> vertices_{
        {rj_geometry::Point{-2.0, 2.0}, rj_geometry::Point{2.0, 2.0}, rj_geometry::Point{2.0, 5.0},
         rj_geometry::Point{-2.0, 5.0}}};
};

}  // namespace strategy