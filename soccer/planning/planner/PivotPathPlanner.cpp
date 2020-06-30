#include "PivotPathPlanner.hpp"

#include <Constants.hpp>
#include <Geometry2d/Pose.hpp>
#include <Geometry2d/Util.hpp>
#include <memory>
#include <planning/primitives/TrapezoidalMotion.hpp>
#include <vector>

#include "planning/Instant.hpp"
#include "planning/Trajectory.hpp"
#include "planning/primitives/AnglePlanning.hpp"
#include "planning/primitives/PathSmoothing.hpp"
#include "planning/primitives/VelocityProfiling.hpp"

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

    double radius = _pivotRadiusMultiplier->value() * Robot_Radius;
    auto pivot_point = command.pivotPoint;

    if (cached_pivot_point.has_value() &&
        cached_pivot_point.value().distTo(pivot_point) < Robot_MouthWidth / 2) {
        pivot_point = cached_pivot_point.value();
        return previous;
    }
    cached_pivot_point = pivot_point;

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

    constexpr double kMaxInterpolationSize = 3 * M_PI / 180;
    const int interpolations =
        std::ceil(std::abs(angle_change) / kMaxInterpolationSize);

    points.push_back(start_instant.position());
    for (int i = 1; i <= interpolations; i++) {
        double percent = (double)i / interpolations;
        double angle = start_angle + angle_change * percent;
        Point point = Point::direction(angle).normalized(radius) + pivot_point;
        points.push_back(point);
    }

    BezierPath pathBezier(points, Point(0, 0), Point(0, 0), linear_constraints);

    Trajectory path =
        ProfileVelocity(pathBezier, start_instant.linear_velocity().mag(), 0,
                        linear_constraints, start_instant.stamp);
    if (Twist::nearly_equals(path.last().velocity, Twist::Zero())) {
        path.HoldFor(RJ::Seconds(3.0));
    }

    AngleFunction function = [pivot_point, pivot_target](
                                 const LinearMotionInstant& instant,
                                 double /*previous_angle*/,
                                 Eigen::Vector2d* jacobian) -> double {
        Point position = instant.position;
        auto angleToPivot = position.angleTo(pivot_point);
        auto angleToPivotTarget = position.angleTo(pivot_target);

        if (abs(angleToPivot - angleToPivotTarget) < DegreesToRadians(10)) {
            return angleToPivotTarget;
        }
        if (jacobian != nullptr) {
            *jacobian = (position - pivot_point).rotate(M_PI / 2);
        }

        return angleToPivot;
    };

    PlanAngles(&path, start_instant, AngleFns::facePoint(pivot_point),
               request.constraints.rot);
    path.stamp(RJ::now());

    previous = path;
    return path;
}

}  // namespace Planning