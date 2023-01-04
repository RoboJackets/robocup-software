#include "planning/planner/path_target_planner.hpp"

#include <utility>

#include <spdlog/spdlog.h>

#include "planning/instant.hpp"
#include "planning/planner/plan_request.hpp"
#include "planning/primitives/velocity_profiling.hpp"
#include "planning/trajectory.hpp"

using namespace rj_geometry;

namespace planning {

Trajectory PathTargetPlanner::plan(const PlanRequest& request) {
    // Collect obstacles
    ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    Trajectory ball_trajectory;
    auto command = std::get<PathTargetCommand>(request.motion_command);
    fill_obstacles(request, &static_obstacles, &dynamic_obstacles, !command.ignore_ball,
                   &ball_trajectory);

    // If we start inside of an obstacle, give up and let another planner take
    // care of it.
    if (static_obstacles.hit(request.start.position())) {
        reset();
        return Trajectory();
    }

    LinearMotionInstant goal_instant = command.goal;
    Point goal_point = goal_instant.position;

    // Cache the start and goal instants for is_done()
    cached_goal_instant_ = goal_instant;
    cached_start_instant_ = request.start.linear_motion();

    AngleFunction angle_function = get_angle_function(request);

    // Call into the sub-object to actually execute the plan.
    Trajectory trajectory = Replanner::create_plan(
        Replanner::PlanParams{request.start, goal_instant, static_obstacles, dynamic_obstacles,
                              request.constraints, angle_function, RJ::Seconds(3.0)},
        std::move(previous_));

    previous_ = trajectory;
    return trajectory;
}

bool PathTargetPlanner::is_done() const {
    if (!cached_start_instant_.has_value() || !cached_goal_instant_.has_value()) {
        return false;
    }

    // TODO(Kevin): also, should enforce the desired angle
    // right now there is a convoluted chain
    // PathTargetPlanner->Replanner->plan_angles which plans angles depending
    // on AngleFunction (either desired face point or desired face angle).
    // nowhere in the chain is there a check if PathTargetPlanner actually is
    // getting to the desired angle.
    //
    // may be related to issue #1506?
    double position_tolerance = 1e-1;
    double velocity_tolerance = 1e-1;
    return LinearMotionInstant::nearly_equals(cached_start_instant_.value(),
                                              cached_goal_instant_.value(), position_tolerance,
                                              velocity_tolerance);
    // TODO(Kevin): in theory this should work as LinearMotionInstant has
    // tolerance built into its == overload, but in practice it doesn't
    /* return cached_start_instant_ == cached_goal_instant_; */
}

AngleFunction PathTargetPlanner::get_angle_function(const PlanRequest& request) {
    auto angle_override = std::get<PathTargetCommand>(request.motion_command).angle_override;

    if (std::holds_alternative<TargetFacePoint>(angle_override)) {
        return AngleFns::face_point(std::get<TargetFacePoint>(angle_override).face_point);
    }

    if (std::holds_alternative<TargetFaceBall>(angle_override)) {
        auto ball_pos = request.world_state->ball.position;
        return AngleFns::face_point(ball_pos);
    }

    if (std::holds_alternative<TargetFaceAngle>(angle_override)) {
        return AngleFns::face_angle(std::get<TargetFaceAngle>(angle_override).target);
    }

    // default to facing tangent to path
    // (rj_convert from ROS to PTMC also follows this default)
    return AngleFns::tangent;
}

}  // namespace planning
