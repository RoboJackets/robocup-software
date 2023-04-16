#pragma once

#include <optional>

#include "planning/planner/path_planner.hpp"
#include "planning/trajectory.hpp"

// subtask planners
#include "collect_path_planner.hpp"
#include "pivot_path_planner.hpp"
#include "settle_path_planner.hpp"

namespace planning {

class PivotKickPathPlanner : public PathPlanner {
public:
    PivotKickPathPlanner() : PathPlanner("pivot_kick") {}

    Trajectory plan(const PlanRequest& plan_request) override;

    void reset() override;
    [[nodiscard]] bool is_done() const override;

private:
    enum State {
        SETTLE,   // slow ball
        COLLECT,  // get ball
        PIVOT,    // orbit to face target
        KICK,     // actuate kicker
        DONE      // tried to kick (maybe missed)
    };
    State current_state_{State::COLLECT};
    State update_state();

    SettlePathPlanner settle_path_planner_{};
    CollectPathPlanner collect_path_planner_{};
    PivotPathPlanner pivot_path_planner_{};
};

}  // namespace planning
