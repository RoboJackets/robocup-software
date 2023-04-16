#pragma once

#include <optional>

#include "planning/planner/path_planner.hpp"
#include "planning/trajectory.hpp"

namespace planning {

class PivotKickPathPlanner : public PathPlanner {
public:
    PivotKickPathPlanner() : PathPlanner("pivot_kick") {}

    Trajectory plan(const PlanRequest& plan_request) override;

    void reset() override;
    [[nodiscard]] bool is_done() const override;

private:
    enum State {
        CAPTURE,  // get ball
        PIVOT,    // orbit to face target
        KICK,     // actuate kicker
        DONE      // tried to kick (maybe missed)
    };
    State current_state_ = State::CAPTURE;
    State update_state();
    bool subskill_is_done_ = false;

    // TODO: this is why composable planners would be good
    Trajectory plan_capture();
    Trajectory plan_pivot();
    Trajectory plan_kick();
};

}  // namespace planning
