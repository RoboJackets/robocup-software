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
    rj_geometry::Point get_idle_pt(const WorldState* world_state, int goalie_id);
    double draw_radius = kRobotRadius;
    QColor draw_color = Qt::black;

private:
    Trajectory previous_{};
    rj_geometry::Point left_goal_post;
    rj_geometry::Point right_goal_post;
    rj_geometry::Point goal_target;
    static constexpr double y_distance_from_goal = 0.1;
    static constexpr double tolerance_for_switching = 0.05;
    bool goalie_positions_initialized = false;
};

}  // namespace planning
