#pragma once
#include <Geometry2d/ShapeSet.hpp>
#include <vector>

#include "Planner.hpp"
#include "planning/primitives/Replanner.hpp"
#include "planning/primitives/VelocityProfiling.hpp"

namespace Planning {

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

    double draw_radius = kRobotRadius;
    QColor draw_color = Qt::black;
    QString draw_layer = "Planning";

private:
    [[nodiscard]] static AngleFunction get_angle_function(
        const PlanRequest& request);

    Trajectory previous_;
};

}  // namespace Planning
