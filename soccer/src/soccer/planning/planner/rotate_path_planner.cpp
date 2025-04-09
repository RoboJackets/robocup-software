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
    if (!cached_angle_change_ && request.trigger_mode == RobotIntent::TriggerMode::AT_END) {
        double target_distance = 
            (request.motion_command.target.position - request.start.pose.position()).mag();
        kIsDoneAngleChangeThresh = max(pow(2, -target_distance), 0.01);
    }
    update_state();
    switch (current_state_) {
        case PIVOT:
            return pivot(request);
        case END:
            return end(request);
    }
    return {};
}

void RotatePathPlanner::update_state() {
    if (!cached_angle_change_) {
        current_state_ = PIVOT;
        return;
    }
    current_state_ = abs(cached_angle_change_.value()) <
                             degrees_to_radians(static_cast<float>(kIsDoneAngleChangeThresh))
                         ? END
                         : PIVOT;
}

// Assumes that we have called plan at least once while in the END state (this assumption should be
// always true given our current planning setup)
bool RotatePathPlanner::is_done() const { return current_state_ == END; }

Trajectory RotatePathPlanner::pivot(const PlanRequest& request) {
    const RobotInstant& start_instant = request.start;
    const auto& linear_constraints = request.constraints.mot;
    const auto& rotation_constraints = request.constraints.rot;

    rj_geometry::ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    fill_obstacles(request, &static_obstacles, &dynamic_obstacles, false);

    const MotionCommand& command = request.motion_command;

    auto pivot_point =
        request.world_state->get_robot(true, static_cast<int>(request.shell_id)).pose.position();
    auto pivot_target = command.target.position;

    double start_angle =
        request.world_state->get_robot(true, static_cast<int>(request.shell_id)).pose.heading();

    double target_angle = pivot_point.angle_to(pivot_target);
    double angle_change = fix_angle_radians(target_angle - start_angle);

    cached_angle_change_ = angle_change;

    Trajectory path{};

    if (cached_target_angle_.has_value() &&
        (*cached_target_angle_ - target_angle) < degrees_to_radians(kIsDoneAngleChangeThresh)) {
        if (cached_path_) {
            path = cached_path_.value();
        } else {
            plan_angles(&path, start_instant, AngleFns::face_point(pivot_target),
                        request.constraints.rot);
            path.stamp(RJ::now());
            cached_path_ = path;
        }
    } else {
        cached_path_.reset();
        plan_angles(&path, start_instant, AngleFns::face_point(pivot_target),
                    request.constraints.rot);
        path.stamp(RJ::now());
        cached_path_ = path;
    }

    cached_target_angle_ = target_angle;

    return path;
}

Trajectory RotatePathPlanner::end(const PlanRequest& request) {
    auto trajectory = Trajectory{};
    trajectory.append_instant(request.start);
    trajectory.mark_angles_valid();
    trajectory.stamp(RJ::now());
    if (request.trigger_mode == RobotIntent::TriggerMode::AT_END) {
        trajectory.trigger_mode = Trajectory::TriggerMode::ON_BREAK_BEAM;
    }
    return trajectory;
}

}  // namespace planning
