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
    if (!cached_angle_change_) {
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

    // SPDLOG_INFO("Pivot point x is {}", pivot_point.x());
    // SPDLOG_INFO("Pivot point y is {}", pivot_point.y());

    double start_angle =
        request.world_state->get_robot(true, static_cast<int>(request.shell_id)).pose.heading();

    double target_angle = pivot_point.angle_to(pivot_target);
    double angle_change = fix_angle_radians(target_angle - start_angle);
    // SPDLOG_INFO("current angle: {}", start_angle);
    // SPDLOG_INFO("cached_target_agne: {}", cached_target_angle_.value_or(0));
    // SPDLOG_INFO("target angle: {}", target_angle);

    cached_angle_change_ = angle_change;
    // SPDLOG_INFO("cached angle change: {}", *cached_angle_change_);


    Trajectory path{};
    auto func = [=](const LinearMotionInstant& instant, double previous_angle,
               Eigen::Vector2d* jacobian) -> double {

                    if ((instant.position - pivot_target).mag() < kRobotRadius) {
                        return previous_angle;
                    }

                    if (jacobian != nullptr) {
                        rj_geometry::Point displacement = pivot_target - instant.position;
                        double distance_sq = displacement.magsq();
                        *jacobian =
                            Eigen::Vector2d(displacement.rotate(-M_PI / 2) / distance_sq);
                    }

                    double delta_forward = instant.position.angle_to(pivot_target);
                    double delta_reverse = fix_angle_radians(M_PI + delta_forward);

                    // double delta_forward = fix_angle_radians(instant.velocity.angle() - previous_angle);
                    // double delta_reverse = fix_angle_radians(M_PI + instant.velocity.angle() - previous_angle);

                    double result = 0.0;

                    if (std::abs(delta_forward) < std::abs(delta_reverse)) {
                        result = delta_forward;
                    } else {
                        result = delta_reverse;
                    }

                    return result;
                };

    if (abs(*cached_target_angle_ - target_angle) < degrees_to_radians(1)) {
        if (cached_path_) {
           path = cached_path_.value();
        }
        else {
            // SPDLOG_INFO("reset");
            // plan_angles(&path, start_instant, AngleFns::face_point(pivot_target), request.constraints.rot);
            plan_angles(&path, start_instant, func, request.constraints.rot);
            path.stamp(RJ::now());
            cached_path_ = path;
        }
    } else {
        // SPDLOG_INFO("reset");
        cached_path_.reset();
        // plan_angles(&path, start_instant, AngleFns::face_point(pivot_target), request.constraints.rot);
        plan_angles(&path, start_instant, func, request.constraints.rot);
        path.stamp(RJ::now());
        cached_path_ = path;
    }

    cached_target_angle_ = target_angle;



    return path;
}

}
