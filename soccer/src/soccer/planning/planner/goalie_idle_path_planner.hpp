#pragma once

#include <spdlog/spdlog.h>

#include "planning/instant.hpp"
#include "planning/planner/path_planner.hpp"
#include "planning/planner/path_target_path_planner.hpp"
#include "planning/primitives/replanner.hpp"
#include "planning/trajectory.hpp"
#include "rj_geometry/point.hpp"
/* #include <rj_msgs/msg/path_target_motion_command.hpp> */
#include "planning/planner/motion_command.hpp"
#include "planning/primitives/angle_planning.hpp"

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
    static rj_geometry::Point get_idle_pt(const WorldState& world_state);

    double draw_radius = kRobotRadius;
    QColor draw_color = Qt::black;

private:
    Trajectory previous_{};
};

}  // namespace planning
