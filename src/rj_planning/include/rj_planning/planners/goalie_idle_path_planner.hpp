#pragma once

#include <spdlog/spdlog.h>

#include <rj_common/planning/instant.hpp>
#include <rj_common/planning/motion_command.hpp>
#include <rj_common/planning/trajectory.hpp>
#include <rj_geometry/point.hpp>

#include "rj_planning/planners/path_planner.hpp"
#include "rj_planning/planners/path_target_path_planner.hpp"
#include "rj_planning/primitives/angle_planning.hpp"
#include "rj_planning/primitives/replanner.hpp"

namespace planning {
/**
 * @brief This planner gives the goalie a way to track the ball when it's not
 * otherwise occupied.
 *
 * Params taken from MotionCommand:
 *   None
 */
class GoalieIdlePathPlanner : public PathPlanner {
public:
    GoalieIdlePathPlanner() : PathPlanner("goalie_idle") {}

    /*
     * From PathPlanner superclass (see path_planner.hpp).
     */
    Trajectory plan(const PlanRequest& plan_request) override;
    void reset() override;
    [[nodiscard]] bool is_done() const override;

    /*
     * @return Point for Goalie to stand in when no shot is coming. Expects
     * ball to be slow.
     */
    static rj_geometry::Point get_idle_pt(const WorldState* world_state, const FieldDimensions* field_dimensions);
    double draw_radius = kRobotRadius;
    QColor draw_color = Qt::black;

private:
    Trajectory previous_{};
};

}  // namespace planning
