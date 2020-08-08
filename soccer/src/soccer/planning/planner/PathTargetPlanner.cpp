#include "planning/planner/PathTargetPlanner.hpp"

#include <utility>

#include "planning/Instant.hpp"
#include "planning/Trajectory.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/primitives/VelocityProfiling.hpp"

using namespace Geometry2d;

namespace Planning {

Trajectory PathTargetPlanner::plan(const PlanRequest& request) {
    // Collect obstacles
    Geometry2d::ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    Trajectory ball_trajectory;
    FillObstacles(request, &static_obstacles, &dynamic_obstacles, true, &ball_trajectory);

    // If we start inside of an obstacle, give up and let another planner take
    // care of it.
    if (static_obstacles.hit(request.start.position())) {
        std::cout << "Hit static obstacle" << std::endl;
        reset();
        return Trajectory();
    }

    auto command = std::get<PathTargetCommand>(request.motionCommand);
    LinearMotionInstant goalInstant = command.goal;
    Point goalPoint = goalInstant.position;

    // Debug drawing
    if (request.debug_drawer != nullptr) {
        request.debug_drawer->drawCircle(goalPoint, static_cast<float>(drawRadius), drawColor,
                                         drawLayer);
    }

    AngleFunction angle_function = getAngleFunction(request);

    // Call into the sub-object to actually execute the plan.
    Trajectory trajectory = Replanner::CreatePlan(
        Replanner::PlanParams{request.start, goalInstant, static_obstacles, dynamic_obstacles,
                              request.constraints, angle_function, RJ::Seconds(3.0)},
        std::move(previous));

    previous = trajectory;
    return trajectory;
}

AngleFunction PathTargetPlanner::getAngleFunction(const PlanRequest& request) {
    auto angle_override = std::get<PathTargetCommand>(request.motionCommand).angle_override;
    if (std::holds_alternative<TargetFacePoint>(angle_override)) {
        return AngleFns::facePoint(std::get<TargetFacePoint>(angle_override).face_point);
    }

    if (std::holds_alternative<TargetFaceAngle>(angle_override)) {
        return AngleFns::faceAngle(std::get<TargetFaceAngle>(angle_override).target);
    }

    return AngleFns::tangent;
}

}  // namespace Planning