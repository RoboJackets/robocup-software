#pragma once
#include <vector>

#include <rj_geometry/shape_set.hpp>

#include "planner.hpp"
#include "planning/instant.hpp"
#include "planning/primitives/replanner.hpp"
#include "planning/primitives/velocity_profiling.hpp"
#include "rj_geometry/pose.hpp"

namespace planning {

class PathTargetPlanner : public PlannerForCommandType<PathTargetCommand> {
public:
    PathTargetPlanner() : PlannerForCommandType("PathTargetPlanner") {}
    ~PathTargetPlanner() override = default;

    PathTargetPlanner(PathTargetPlanner&&) noexcept = default;
    PathTargetPlanner& operator=(PathTargetPlanner&&) noexcept = default;
    PathTargetPlanner(const PathTargetPlanner&) = default;
    PathTargetPlanner& operator=(const PathTargetPlanner&) = default;

    Trajectory plan(const PlanRequest& request) override;
    void reset() override { previous_ = Trajectory(); }

    bool is_done() const override;

    // vars to tell if is_done
    std::optional<LinearMotionInstant> cached_start_instant_;
    std::optional<LinearMotionInstant> cached_goal_instant_;

    double draw_radius = kRobotRadius;
    QColor draw_color = Qt::black;

private:
    [[nodiscard]] static AngleFunction get_angle_function(
        const PlanRequest& request);

    Trajectory previous_;
};

}  // namespace planning
