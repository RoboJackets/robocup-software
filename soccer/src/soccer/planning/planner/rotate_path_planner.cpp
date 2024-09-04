#include "rotate_path_planner.hpp"

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

Trajectory RotatePathPlanner::plan(const PlanRequest& request) {
    return pivot(request); // type is Trajectory
}

bool RotatePathPlanner::is_done() const {
    if (!cached_angle_change_.has_value()) {
        return false;
    }
    return abs(cached_angle_change_.value()) < degrees_to_radians(static_cast<float>(kIsDoneAngleChangeThresh));
}

Trajectory RotatePathPlanner::pivot(const PlanRequest& request) {
    const RobotInstant& start_instant = request.start;
    const auto& linear_constraints = request.constraints.mot;
    const auto& rotation_constraints = request.constraints.rot;

    rj_geometry::ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    fill_obstacles(request, &static_obstacles, &dynamic_obstacles, false);

    const MotionCommand& command = request.motion_command;

    auto pivot_point = request.world_state->get_robot(true, static_cast<int>(request.shell_id)).pose.position();
    auto pivot_target = command.target.position;

    SPDLOG_INFO("Pivot point x is {}", pivot_point.x());
    SPDLOG_INFO("Pivot point y is {}", pivot_point.y());

    auto final_position = pivot_point;
    // SPDLOG_INFO("Final position x is {}", final_position.x());
    // SPDLOG_INFO("Final position y is {}", final_position.y());

    std::vector<Point> points;

    double start_angle = 
        request.world_state->get_robot(true, static_cast<int>(request.shell_id)).pose.heading();

    double target_angle = pivot_point.angle_to(pivot_target);
    double angle_change = fix_angle_radians(target_angle - start_angle);
    SPDLOG_INFO("target anlge: {}", target_angle);
    SPDLOG_INFO("current angle: {}", start_angle);

    cached_angle_change_ = angle_change;

    constexpr double kMaxInterpolationRadius = 10 * M_PI / 100;
    const int interpolations = std::ceil(std::abs(angle_change) / kMaxInterpolationRadius);

    Trajectory path{};
    // path.append_instant(start_instant);
    // for (int i = 1; i <= interpolations; i++) {
    //     auto instant = RobotInstant{Pose{start_instant.position(), }};
    //     path.append_instant(instant);
    // }
    // SPDLOG_INFO("leng: {}", points.size());

    // BezierPath path_bezier(points, Point(0, 0), Point(0, 0), linear_constraints);

    // Trajectory path = profile_velocity(path_bezier, start_instant.linear_velocity().mag(), 0, linear_constraints, start_instant.stamp);

    // if (!Twist::nearly_equals(path.last().velocity, Twist::zero())) {
    //     RobotInstant last;
    //     last.position() = final_position;
    //     last.velocity = Twist::zero();
    //     last.stamp = path.end_time() + 100ms;
    //     path.append_instant(last);
    // }

    plan_angles(&path, start_instant, AngleFns::face_point(pivot_target), request.constraints.rot);
    path.stamp(RJ::now());

    return path;
}

}