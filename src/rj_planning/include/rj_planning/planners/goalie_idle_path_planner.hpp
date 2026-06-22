#pragma once

#include <cmath>

#include <spdlog/spdlog.h>

#include <rj_common/planning/instant.hpp>
#include <rj_common/planning/motion_command.hpp>
#include <rj_common/planning/trajectory.hpp>
#include <rj_common/time.hpp>
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
     * @return Point for the goalie to idle at when no shot is coming. Sweeps
     * back and forth across the goal mouth as a function of time.
     */
    static rj_geometry::Point get_idle_pt(const FieldDimensions* field_dimensions);

    double draw_radius = kRobotRadius;
    QColor draw_color = Qt::black;

private:
    Trajectory previous_{};

    // How fast the goalie sweeps across the goal mouth, in rad/s of the
    // underlying sine wave (higher = faster pacing).
    static constexpr double kSweepRate = 1.2;
    // How far in front of the goal line (m) the goalie idles.
    static constexpr double kGoalLineOffset = 0.2;
    // Fraction of the goal width the goalie sweeps across (0-1). 1.0 sweeps
    // all the way to each post; smaller values keep it nearer the center.
    static constexpr double kSweepFraction = 0.75;
};

}  // namespace planning
