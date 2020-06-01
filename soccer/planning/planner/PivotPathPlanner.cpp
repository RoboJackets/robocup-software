#include "PivotPathPlanner.hpp"

#include <Constants.hpp>
#include <Geometry2d/Pose.hpp>
#include <Geometry2d/Util.hpp>
#include <memory>
#include <vector>

#include "planning/Instant.hpp"
#include "planning/Trajectory.hpp"
#include "planning/low_level/PathSmoothing.hpp"
#include "planning/low_level/VelocityProfiling.hpp"

namespace Planning {
using namespace Geometry2d;

REGISTER_CONFIGURABLE(PivotPathPlanner);

ConfigDouble* PivotPathPlanner::_pivotRadiusMultiplier;

void PivotPathPlanner::createConfiguration(Configuration* cfg) {
    _pivotRadiusMultiplier =
        new ConfigDouble(cfg, "Pivot/radius", 1.0,
                         "Multiplier for the pivotRadius. PivotRadius = "
                         "RobotRadius * multiplier");
}

Trajectory PivotPathPlanner::plan(PlanRequest&& request) {
    const RobotInstant& startInstant = request.start;
    const auto& motionConstraints = request.constraints.mot;
    const auto& rotationConstraints = request.constraints.rot;

    Geometry2d::ShapeSet staticObstacles;
    std::vector<DynamicObstacle> dynamicObstacles;
    FillObstacles(request, &staticObstacles, &dynamicObstacles, false);

    const auto& command =
        std::get<PivotCommand>(request.motionCommand);

    if (!shouldReplan(command)) {
        return previous;
    }

    double radius = _pivotRadiusMultiplier->value() * Robot_Radius;
    auto pivotPoint = command.pivotPoint;
    auto pivotTarget = command.pivotTarget;
    auto endTarget =
        pivotPoint + (pivotPoint - pivotTarget).normalized(radius);
    std::vector<Point> points;

    // maxSpeed = maxRadians * radius
    MotionConstraints newConstraints = request.constraints.mot;
    newConstraints.maxSpeed =
        std::min(newConstraints.maxSpeed,
                 rotationConstraints.maxSpeed * radius) *
        .5;

    double startAngle = pivotPoint.angleTo(startInstant.position());
    double targetAngle = pivotPoint.angleTo(endTarget);
    double change = fixAngleRadians(targetAngle - startAngle);

    const int interpolations = 10;

    points.push_back(startInstant.position());
    for (int i = 1; i <= interpolations; i++) {
        double percent = (double)i / interpolations;
        double angle = startAngle + change * percent;
        Point point =
            Point::direction(angle).normalized(radius) + pivotPoint;
        points.push_back(point);
    }

    BezierPath pathBezier(points, startInstant.linear_velocity(),
                          Point(0, 0), motionConstraints);

    Trajectory path = ProfileVelocity(
        pathBezier,
        startInstant.linear_velocity().mag(),
        0,
        motionConstraints,
        startInstant.stamp);

    AngleFunction function =
        [pivotPoint, pivotTarget] (const RobotInstant& instant) -> double {
            Point position = instant.position();
            auto angleToPivot = position.angleTo(pivotPoint);
            auto angleToPivotTarget = position.angleTo(pivotTarget);

            if (abs(angleToPivot - angleToPivotTarget) <
                DegreesToRadians(10)) {
                return angleToPivotTarget;
            }

            return angleToPivot;
        };

    PlanAngles(path, startInstant,
               AngleFns::facePoint(pivotPoint), request.constraints.rot);

    previous = path;
    return path;
}

bool PivotPathPlanner::shouldReplan(const PivotCommand& command) const {
    if (previous.empty()) {
        return true;
    }

    // Calculate the endpoint of this maneuver.
    RobotInstant start = previous.first();
    auto pivotPoint = command.pivotPoint;
    auto pivotTarget = command.pivotTarget;
    double radius = _pivotRadiusMultiplier->value() * Robot_Radius;
    auto endTarget =
        pivotPoint + (pivotPoint - pivotTarget).normalized(radius);
    double targetChange = (previous.last().position() - endTarget).mag();

    // If the target has changed significantly, we need to replan.
    if (targetChange > 0.1) {
        return true;
    }

    // Finally, if the previous trajectory ended more than 0.5s ago, replan.
    return previous.end_time() + RJ::Seconds(0.5) < RJ::now();
}

}  // namespace Planning