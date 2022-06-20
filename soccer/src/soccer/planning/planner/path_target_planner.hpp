#pragma once
#include <rj_geometry/shape_set.hpp>
#include <vector>

#include "planner.hpp"
#include "planning/primitives/replanner.hpp"
#include "planning/primitives/velocity_profiling.hpp"

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

    double draw_radius = kRobotRadius;
    QColor draw_color = Qt::black;

private:
    [[nodiscard]] static AngleFunction get_angle_function(
        const PlanRequest& request);

    Trajectory previous_;
};

}  // namespace planning
