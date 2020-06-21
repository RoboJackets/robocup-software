#include "PivotPathPlanner.hpp"

#include <Constants.hpp>
#include <Geometry2d/Pose.hpp>
#include <Geometry2d/Util.hpp>
#include <memory>
#include <planning/low_level/TrapezoidalMotion.hpp>
#include <vector>

#include "planning/Instant.hpp"
#include "planning/Trajectory.hpp"
#include "planning/low_level/AnglePlanning.hpp"
#include "planning/low_level/PathSmoothing.hpp"
#include "planning/low_level/VelocityProfiling.hpp"

namespace Planning {
using namespace Geometry2d;

REGISTER_CONFIGURABLE(PivotPathPlanner);

ConfigDouble* PivotPathPlanner::_pivotRadiusMultiplier;

void PivotPathPlanner::createConfiguration(Configuration* cfg) {
    // NOLINTNEXTLINE
    _pivotRadiusMultiplier =
        new ConfigDouble(cfg, "PathPlanner/Pivot/radiusMultiplier", 1.0,
                         "Multiplier for the pivotRadius. PivotRadius = "
                         "RobotRadius * multiplier");
}

Trajectory PivotPathPlanner::plan(const PlanRequest& request) {
    const RobotInstant& start_instant = request.start;
    const auto& linear_constraints = request.constraints.mot;
    const auto& rotation_constraints = request.constraints.rot;

    Geometry2d::ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    FillObstacles(request, &static_obstacles, &dynamic_obstacles, false);

    const auto& command = std::get<PivotCommand>(request.motionCommand);

    if (!shouldReplan(command)) {
        return previous;
    }

    double radius = _pivotRadiusMultiplier->value() * Robot_Radius;
    auto pivot_point = command.pivotPoint;
    auto pivot_target = command.pivotTarget;
    auto final_position =
        pivot_point + (pivot_point - pivot_target).normalized(radius);
    std::vector<Point> points;

    // maxSpeed = maxRadians * radius
    MotionConstraints new_constraints = request.constraints.mot;
    new_constraints.maxSpeed =
        std::min(new_constraints.maxSpeed,
                 rotation_constraints.maxSpeed * radius) *
        .5;

    double start_angle = pivot_point.angleTo(start_instant.position());
    double target_angle = pivot_point.angleTo(final_position);
    double angle_change = fixAngleRadians(target_angle - start_angle);

    const int interpolations = 10;

    points.push_back(start_instant.position());
    for (int i = 1; i <= interpolations; i++) {
        double percent = (double)i / interpolations;
        double angle = start_angle + angle_change * percent;
        Point point = Point::direction(angle).normalized(radius) + pivot_point;
        points.push_back(point);
    }

    BezierPath pathBezier(points, start_instant.linear_velocity(), Point(0, 0),
                          linear_constraints);

    Trajectory path =
        ProfileVelocity(pathBezier, start_instant.linear_velocity().mag(), 0,
                        linear_constraints, start_instant.stamp);

    AngleFunction function = [pivot_point, pivot_target](
                                 const LinearMotionInstant& instant,
                                 double /*previous_angle*/, Eigen::Vector2d *
                                 /*jacobian*/) -> double {
        Point position = instant.position;
        auto angleToPivot = position.angleTo(pivot_point);
        auto angleToPivotTarget = position.angleTo(pivot_target);

        if (abs(angleToPivot - angleToPivotTarget) < DegreesToRadians(10)) {
            return angleToPivotTarget;
        }

        return angleToPivot;
    };

    PlanAngles(&path, start_instant, AngleFns::facePoint(pivot_point),
               request.constraints.rot);
    path.stamp(RJ::now());

    previous = path;
    return path;
}

bool PivotPathPlanner::shouldReplan(const PivotCommand& command) const {
    if (previous.empty()) {
        return true;
    }

    // Calculate the endpoint of this maneuver.
    RobotInstant start = previous.first();
    RobotInstant end = previous.last();

    Point target_point = command.pivotTarget;
    Point pivot_point = command.pivotPoint;

    // In addition, we should be facing the right way at the end.
    Point face_point =
        end.position() + Point::direction(end.heading())
                             .normalized((target_point - end.position()).mag());

    return face_point.distTo(target_point) > 0.05;
}

}  // namespace Planning