#include "line_pivot_path_planner.hpp"

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

Trajectory LinePivotPathPlanner::plan(const PlanRequest& request) {
    current_state_ = next_state(request);
    Trajectory path;

    if (current_state_ == LINE) {
        path = line(request);
    } else {
        path = pivot(request);
    }

    return path;
}

bool LinePivotPathPlanner::is_done() const {
    if (!cached_angle_change_.has_value()) {
        return false;
    }
    return abs(cached_angle_change_.value()) <
           degrees_to_radians(static_cast<float>(kIsDoneAngleChangeThresh));
}

LinePivotPathPlanner::State LinePivotPathPlanner::next_state(const PlanRequest& plan_request) {
    const MotionCommand& command = plan_request.motion_command;
    auto current_point =
        plan_request.world_state->get_robot(true, static_cast<int>(plan_request.shell_id))
            .pose.position();
    auto pivot_point = command.pivot_point;

    double dist_from_point = command.pivot_radius;
    auto target_point = pivot_point + (current_point - pivot_point).normalized(dist_from_point);
    double vel = plan_request.world_state->get_robot(true, static_cast<int>(plan_request.shell_id))
                     .velocity.linear()
                     .mag();
    if (current_state_ == LINE && (target_point.dist_to(current_point) < 0.3) && (vel < 0.3) &&
        (!plan_request.play_state.is_stop())) {
        return PIVOT;
    }
    if (current_state_ == PIVOT && (pivot_point.dist_to(current_point) > (dist_from_point * 5))) {
        return LINE;
    }
    return current_state_;
}

Trajectory LinePivotPathPlanner::line(const PlanRequest& plan_request) {
    const MotionCommand& command = plan_request.motion_command;
    auto pivot_point = command.pivot_point;
    auto current_point =
        plan_request.world_state->get_robot(true, static_cast<int>(plan_request.shell_id))
            .pose.position();

    double dist_from_point = command.pivot_radius;
    auto target_point = pivot_point + (current_point - pivot_point).normalized(dist_from_point);

    // Create an updated MotionCommand and forward to PathTargetPathPlaner
    PlanRequest modified_request = plan_request;

    LinearMotionInstant target{target_point};

    MotionCommand modified_command{"path_target", target};
    modified_command.ignore_ball = true;
    modified_request.motion_command = modified_command;

    return path_target_.plan(modified_request);
}

Trajectory LinePivotPathPlanner::pivot(const PlanRequest& request) {
    const RobotInstant& start_instant = request.start;
    const auto& linear_constraints = request.constraints.mot;
    const auto& rotation_constraints = request.constraints.rot;

    rj_geometry::ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    fill_obstacles(request, &static_obstacles, &dynamic_obstacles, false);

    const MotionCommand& command = request.motion_command;

    double radius = pivot::PARAM_radius_multiplier * command.pivot_radius;
    auto pivot_point = command.pivot_point;
    auto pivot_target = command.target.position;

    auto final_position = pivot_point + (pivot_point - pivot_target).normalized(radius);
    std::vector<Point> points;

    // max_speed = max_radians * radius
    MotionConstraints new_constraints = request.constraints.mot;
    new_constraints.max_speed =
        std::min(new_constraints.max_speed, rotation_constraints.max_speed * radius) * .5;

    double start_angle = pivot_point.angle_to(
        request.world_state->get_robot(true, static_cast<int>(request.shell_id)).pose.position());
    double target_angle = pivot_point.angle_to(final_position);
    double angle_change = fix_angle_radians(target_angle - start_angle);

    cached_angle_change_ = angle_change;

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

    plan_angles(&path, start_instant, AngleFns::face_point(pivot_point), request.constraints.rot);
    path.stamp(RJ::now());

    return path;
}

}  // namespace planning
