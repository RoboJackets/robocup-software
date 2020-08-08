#include "PivotPathPlanner.hpp"

#include <memory>
#include <vector>

#include <Geometry2d/Pose.hpp>
#include <Geometry2d/Util.hpp>
#include <rj_constants/constants.hpp>

#include "planning/Instant.hpp"
#include "planning/Trajectory.hpp"
#include "planning/primitives/AnglePlanning.hpp"
#include "planning/primitives/PathSmoothing.hpp"
#include "planning/primitives/TrapezoidalMotion.hpp"
#include "planning/primitives/VelocityProfiling.hpp"

namespace Planning {
using namespace Geometry2d;

REGISTER_CONFIGURABLE(PivotPathPlanner);

ConfigDouble* PivotPathPlanner::pivot_radius_multiplier;

void PivotPathPlanner::create_configuration(Configuration* cfg) {
    // NOLINTNEXTLINE
    pivot_radius_multiplier = new ConfigDouble(cfg, "PathPlanner/Pivot/radiusMultiplier", 1.0,
                                               "Multiplier for the pivotRadius. PivotRadius = "
                                               "RobotRadius * multiplier");
}

Trajectory PivotPathPlanner::plan(const PlanRequest& request) {
    const RobotInstant& start_instant = request.start;
    const auto& linear_constraints = request.constraints.mot;
    const auto& rotation_constraints = request.constraints.rot;

    Geometry2d::ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    fill_obstacles(request, &static_obstacles, &dynamic_obstacles, false);

    const auto& command = std::get<PivotCommand>(request.motion_command);

    double radius = pivot_radius_multiplier->value() * kRobotRadius;
    auto pivot_point = command.pivot_point;

    if (cached_pivot_point_.has_value() &&
        cached_pivot_point_.value().dist_to(pivot_point) < kRobotMouthWidth / 2) {
        pivot_point = cached_pivot_point_.value();
        return previous_;
    }
    cached_pivot_point_ = pivot_point;

    auto pivot_target = command.pivot_target;
    auto final_position = pivot_point + (pivot_point - pivot_target).normalized(radius);
    std::vector<Point> points;

    // max_speed = max_radians * radius
    MotionConstraints new_constraints = request.constraints.mot;
    new_constraints.max_speed =
        std::min(new_constraints.max_speed, rotation_constraints.max_speed * radius) * .5;

    double start_angle = pivot_point.angle_to(start_instant.position());
    double target_angle = pivot_point.angle_to(final_position);
    double angle_change = fix_angle_radians(target_angle - start_angle);

    constexpr double kMaxInterpolationSize = 3 * M_PI / 180;
    const int interpolations = std::ceil(std::abs(angle_change) / kMaxInterpolationSize);

    points.push_back(start_instant.position());
    for (int i = 1; i <= interpolations; i++) {
        double percent = (double)i / interpolations;
        double angle = start_angle + angle_change * percent;
        Point point = Point::direction(angle).normalized(radius) + pivot_point;
        points.push_back(point);
    }

    BezierPath path_bezier(points, Point(0, 0), Point(0, 0), linear_constraints);

    Trajectory path = profile_velocity(path_bezier, start_instant.linear_velocity().mag(), 0,
                                       linear_constraints, start_instant.stamp);
    if (Twist::nearly_equals(path.last().velocity, Twist::zero())) {
        path.hold_for(RJ::Seconds(3.0));
    }

    AngleFunction function = [pivot_point, pivot_target](const LinearMotionInstant& instant,
                                                         double /*previous_angle*/,
                                                         Eigen::Vector2d* jacobian) -> double {
        Point position = instant.position;
        auto angle_to_pivot = position.angle_to(pivot_point);
        auto angle_to_pivot_target = position.angle_to(pivot_target);

        if (abs(angle_to_pivot - angle_to_pivot_target) < degrees_to_radians(10)) {
            return angle_to_pivot_target;
        }
        if (jacobian != nullptr) {
            *jacobian = (position - pivot_point).rotate(M_PI / 2);
        }

        return angle_to_pivot;
    };

    plan_angles(&path, start_instant, AngleFns::face_point(pivot_point), request.constraints.rot);
    path.stamp(RJ::now());

    previous_ = path;
    return path;
}

}  // namespace Planning