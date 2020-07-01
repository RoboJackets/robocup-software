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
    void reset() override { previous = Trajectory(); }

    double drawRadius = Robot_Radius;
    QColor drawColor = Qt::black;
    QString drawLayer = "Planning";

private:
    [[nodiscard]] static AngleFunction getAngleFunction(
        const PlanRequest& request);

    Trajectory previous;
};

}  // namespace Planning
