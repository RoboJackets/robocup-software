#include "planning/planner/path_target_planner.hpp"

#include <utility>

#include "planning/instant.hpp"
#include "planning/trajectory.hpp"
#include "planning/planner/plan_request.hpp"
#include "planning/primitives/velocity_profiling.hpp"

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

    // Debug drawing
    if (request.debug_drawer != nullptr) {
        // TODO(Kevin): delete debug_drawer.hpp and see what happens, since we use
        // ros_debug_drawer.hpp and that's incredibly confusing
        request.debug_drawer->draw_circle(Circle(goal_point, static_cast<float>(draw_radius)),
                                          draw_color);
        // TODO: DELETE ME!
        Point pt_a = Point(2.0, 0.5);
        Point pt_b = Point(2.0, 8.5);
        request.debug_drawer->draw_segment(Segment(pt_a, pt_b));
    }

    AngleFunction angle_function = get_angle_function(request);

    // Call into the sub-object to actually execute the plan.
    Trajectory trajectory = Replanner::create_plan(
        Replanner::PlanParams{request.start, goal_instant, static_obstacles, dynamic_obstacles,
                              request.constraints, angle_function, RJ::Seconds(3.0)},
        std::move(previous_), request.debug_drawer);

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

}  // namespace planning
