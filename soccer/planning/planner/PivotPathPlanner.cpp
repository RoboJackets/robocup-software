#include "PivotPathPlanner.hpp"

#include <Constants.hpp>
#include <Geometry2d/Pose.hpp>
#include <Geometry2d/Util.hpp>
#include <memory>
#include <vector>

#include "planning/trajectory/PathSmoothing.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"

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

    float startAngle = pivotPoint.angleTo(startInstant.pose.position());
    float targetAngle = pivotPoint.angleTo(endTarget);
    float change = fixAngleRadians(targetAngle - startAngle);

    const int interpolations = 10;

    points.push_back(startInstant.pose.position());
    for (int i = 1; i <= interpolations; i++) {
        float percent = (float)i / interpolations;
        float angle = startAngle + change * percent;
        Point point =
            Point::direction(angle).normalized(radius) + pivotPoint;
        points.push_back(point);
    }

    BezierPath postBezier(points, startInstant.velocity.linear(),
                          Point(0, 0), motionConstraints);

    Trajectory path = ProfileVelocity(
        postBezier, startInstant.velocity.linear().mag(),
        0, motionConstraints, startInstant.stamp);

    AngleFunction function =
        [pivotPoint, pivotTarget] (const RobotInstant& instant) -> double {
            Point position = instant.pose.position();
            auto angleToPivot = position.angleTo(pivotPoint);
            auto angleToPivotTarget = position.angleTo(pivotTarget);

            if (abs(angleToPivot - angleToPivotTarget) <
                DegreesToRadians(10)) {
                return angleToPivotTarget;
            } else {
                return angleToPivot;
            }
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
    double targetChange = (previous.last().pose.position() - endTarget).mag();

    // If the target has changed significantly, we need to replan.
    if (targetChange > 0.1) {
        return true;
    }

    // Finally, if the previous trajectory ended more than 0.5s ago, replan.
    return previous.end_time() + RJ::Seconds(0.5) < RJ::now();
}

}  // namespace Planning