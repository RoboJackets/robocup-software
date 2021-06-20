#include "pivot_path_planner.hpp"

#include <memory>
#include <vector>

#include <rj_constants/constants.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_geometry/util.hpp>

#include "planning/instant.hpp"
#include "planning/planning_params.hpp"
#include "planning/primitives/angle_planning.hpp"
#include "planning/primitives/path_smoothing.hpp"
#include "planning/primitives/trapezoidal_motion.hpp"
#include "planning/primitives/velocity_profiling.hpp"
#include "planning/trajectory.hpp"

namespace planning {
using namespace rj_geometry;

Trajectory PivotPathPlanner::plan(const PlanRequest& request) {
    const RobotInstant& start_instant = request.start;
    const auto& linear_constraints = request.constraints.mot;
    const auto& rotation_constraints = request.constraints.rot;

    rj_geometry::ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    fill_obstacles(request, &static_obstacles, &dynamic_obstacles, false);

    const auto& command = std::get<PivotCommand>(request.motion_command);

    double radius = pivot::PARAM_radius_multiplier * kRobotRadius;
    auto pivot_point = command.pivot_point;
    auto pivot_target = command.pivot_target;

    const bool pivot_target_unchanged =
        cached_pivot_target_.has_value() &&
        cached_pivot_target_.value().dist_to(pivot_target) < kRobotMouthWidth / 2;
    bool pivot_point_unchanged =
        cached_pivot_point_.has_value() &&
        cached_pivot_point_.value().dist_to(pivot_point) < kRobotMouthWidth / 2;

    // TODO(Kyle): These need real constants
    if (pivot_target_unchanged && pivot_point_unchanged) {
        return previous_;
    }

    if (pivot_point_unchanged) {
        pivot_point = *cached_pivot_point_;
    }
    if (pivot_target_unchanged) {
        pivot_target = *cached_pivot_target_;
    }

    cached_pivot_target_ = pivot_target;
    cached_pivot_point_ = pivot_point;

    auto final_position = pivot_point + (pivot_point - pivot_target).normalized(radius);
    std::vector<Point> points;

    // max_speed = max_radians * radius
    MotionConstraints new_constraints = request.constraints.mot;
    new_constraints.max_speed =
        std::min(new_constraints.max_speed, rotation_constraints.max_speed * radius) * .5;

    double start_angle = pivot_point.angle_to(start_instant.position());
    double target_angle = pivot_point.angle_to(final_position);
    double angle_change = fix_angle_radians(target_angle - start_angle);

    constexpr double kMaxInterpolationRadians = 10 * M_PI / 180;
    const int interpolations = std::ceil(std::abs(angle_change) / kMaxInterpolationRadians);

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
    if (!Twist::nearly_equals(path.last().velocity, Twist::zero())) {
        RobotInstant last;
        last.position() = final_position;
        last.velocity = Twist::zero();
        last.stamp = path.end_time() + 100ms;
        path.append_instant(last);
    }
    path.hold_for(RJ::Seconds(3.0));

    AngleFunction function = [pivot_point, pivot_target](const LinearMotionInstant& instant,
                                                         double /*previous_angle*/,
                                                         Eigen::Vector2d* jacobian) -> double {
        Point position = instant.position;
        auto angle_to_pivot_target = position.angle_to(pivot_target);
        auto angle_to_pivot = position.angle_to(pivot_point);

        Point target_point = pivot_point;
        if (abs(angle_to_pivot - angle_to_pivot_target) < degrees_to_radians(10)) {
            target_point = pivot_target;
        }

        auto angle_to_target = position.angle_to(target_point);

        if (jacobian != nullptr) {
            // The angle to the point changes with the dot product of the tangent vector to the
            // circle with the robot's velocity, divided by the radius. Therefore, the rotated
            // vector needs to be divided by the radius twice to get the jacobian.
            *jacobian = (position - target_point).rotate(M_PI / 2);
            *jacobian /= jacobian->squaredNorm();
        }

        return angle_to_target;
    };

    plan_angles(&path, start_instant, AngleFns::face_point(pivot_point), request.constraints.rot);
    path.stamp(RJ::now());

    previous_ = path;
    return path;
}

}  // namespace planning