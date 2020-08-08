#include "planning/planner/path_target_planner.hpp"

#include <utility>

#include "planning/instant.hpp"
#include "planning/trajectory.hpp"
#include "planning/planner/plan_request.hpp"
#include "planning/primitives/velocity_profiling.hpp"

using namespace rj_geometry;

namespace Planning {

Trajectory PathTargetPlanner::plan(const PlanRequest& request) {
    // Collect obstacles
    rj_geometry::ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    Trajectory ball_trajectory;
    fill_obstacles(request, &static_obstacles, &dynamic_obstacles, true, &ball_trajectory);

    // If we start inside of an obstacle, give up and let another planner take
    // care of it.
    if (static_obstacles.hit(request.start.position())) {
        std::cout << "Hit static obstacle" << std::endl;
        reset();
        return Trajectory();
    }

    auto command = std::get<PathTargetCommand>(request.motion_command);
    LinearMotionInstant goal_instant = command.goal;
    Point goal_point = goal_instant.position;

    // Debug drawing
    if (request.debug_drawer != nullptr) {
        request.debug_drawer->draw_circle(goal_point, static_cast<float>(draw_radius), draw_color,
                                          draw_layer);
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

}  // namespace Planning