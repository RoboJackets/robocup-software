#pragma once

#include <spdlog/spdlog.h>

#include <rj_geometry/point.hpp>

#include "planner.hpp"
#include "planning/primitives/replanner.hpp"

namespace planning {

/**
 * Planner which tries to block a shot on goal as quickly as possible
 * Whether this means moving and stopping in the path of the ball
 * or completely driving through and "slapping" the ball.
 *
 * (More accurate name would be "GoalieBlock", but Intercept kept for legacy
 * reasons.)
 */

class InterceptPlanner : public PlannerForCommandType<InterceptCommand> {
public:
    InterceptPlanner() : PlannerForCommandType<InterceptCommand>("intercept"){};

    Trajectory plan(const PlanRequest& request) override;

    [[nodiscard]] bool is_done() const override;

    /*
     * @return if a shot on goal is coming
     *
     * Static so it can be used in Goalie's FSM.
     */
    static bool shot_on_goal_detected(const WorldState* world_state);

    double draw_radius = kRobotRadius;
    QColor draw_color = Qt::black;

private:
    bool is_done_ = false;

    /*
     * @return Point for Goalie to block a shot. Assumes ball is not slow.
     *
     * TODO(Kevin): make 1 common param for "ball slow/fast" for GoalieIdle and
     * this planner
     */
    std::optional<rj_geometry::Point> get_block_pt(const WorldState* world_state);

    Trajectory previous_{};
};
}  // namespace planning
