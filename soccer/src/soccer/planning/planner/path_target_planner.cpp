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

    // Debug drawing
    if (request.debug_drawer != nullptr) {
        request.debug_drawer->draw_circle(Circle(goal_point, static_cast<float>(draw_radius)),
                                          draw_color);
    }

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

    // maximum difference in position and velocity that we can still
    // consider close enough (in m)
    // TODO(#1913): connect gameplay to planner is_done to avoid two diff threshold params
    double temp_correction = 1.2;  // be X% more generous than gameplay so we can see change

    // TODO(Kevin): also, should enforce the desired angle
    // right now there is a convoluted chain
    // PathTargetPlanner->Replanner->plan_angles which plans angles depending
    // on AngleFunction (either desired face point or desired face angle).
    // nowhere in the chain is there a check if PathTargetPlanner actually is
    // getting to the desired angle.
    //
    // may be related to issue #1506?
    double position_tolerance = 1e-2 * temp_correction;
    double velocity_tolerance = 1e-1 * temp_correction;
    return LinearMotionInstant::nearly_equals(cached_start_instant_.value(),
                                              cached_goal_instant_.value(), position_tolerance,
                                              velocity_tolerance);
}

AngleFunction PathTargetPlanner::get_angle_function(const PlanRequest& request) {
    auto angle_override = std::get<PathTargetCommand>(request.motion_command).angle_override;
    if (std::holds_alternative<TargetFacePoint>(angle_override)) {
        return AngleFns::face_point(std::get<TargetFacePoint>(angle_override).face_point);
    }

    if (std::holds_alternative<TargetFaceAngle>(angle_override)) {
        return AngleFns::face_angle(std::get<TargetFaceAngle>(angle_override).target);
    }

    return AngleFns::tangent;
}

}  // namespace planning
