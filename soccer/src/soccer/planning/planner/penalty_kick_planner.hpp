#pragma once

#include <spdlog/spdlog.h>

#include "planning/instant.hpp"
#include "planning/planner/path_target_planner.hpp"
#include "planning/planner/planner.hpp"
#include "planning/primitives/replanner.hpp"
#include "planning/trajectory.hpp"
#include "rj_geometry/point.hpp"
/* #include <rj_msgs/msg/path_target_motion_command.hpp> */
#include "planning/planner/motion_command.hpp"
#include "planning/primitives/angle_planning.hpp"

namespace planning {
/**
 * @brief This planner creates penalty kick behavior:
 * (1) dribble the ball until close to the goal (legal in
 * PK)
 * (2) shoot ball at open side of goal
 */
class PenaltyKickPlanner : public PlannerForCommandType<PenaltyKickMotionCommand> {
public:
    PenaltyKickPlanner() : PlannerForCommandType<PenaltyKickMotionCommand>("penalty_kick") {}

    /*
     * From Planner superclass (see planner.hpp).
     */
    Trajectory plan(const PlanRequest& plan_request) override;
    void reset() override;
    [[nodiscard]] bool is_done() const override;

private:
    Trajectory previous_{};
};

}  // namespace planning
