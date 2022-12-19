#pragma once

#include "planner.hpp"

namespace planning {

/**
 * @brief Planner for Goalie to block shots on goal.
 *
 * Tries to intercept the path ball as quickly as possible, whether this means
 * moving and stopping in the path of the ball or completely driving through
 * and "slapping" the ball.
 *
 * (Intercept is a legacy name from when this planner was designed for both
 * Goalie and Defenders; BlockShotPlanner would be a better name now.)
 */

class InterceptPlanner : public PlannerForCommandType<InterceptCommand> {
public:
    InterceptPlanner()
        : PlannerForCommandType<InterceptCommand>("InterceptPlanner"){};

    Trajectory plan(const PlanRequest& request) override;

    [[nodiscard]] bool is_done() const override;
};
}  // namespace planning
