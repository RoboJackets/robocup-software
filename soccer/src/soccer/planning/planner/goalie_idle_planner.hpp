#pragma once

#include "planning/instant.hpp"
#include "planning/planner/planner.hpp"
#include "planning/primitives/replanner.hpp"
#include "planning/trajectory.hpp"
#include "rj_geometry/point.hpp"

namespace planning {
/**
 * @brief This planner gives the goalie a way to track the ball when it's not
 * otherwise occupied.
 */
class GoalieIdlePlanner : public PlannerForCommandType<GoalieIdleCommand> {
public:
    GoalieIdlePlanner() : PlannerForCommandType<GoalieIdleCommand>("goalie_idle") {}

    Trajectory plan(const PlanRequest& plan_request) override;

    void reset() override;
    [[nodiscard]] bool is_done() const override;

private:
};

}  // namespace planning
