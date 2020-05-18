#include "planning/planner/PathTargetPlanner.hpp"

#include <utility>
#include "EscapeObstaclesPathPlanner.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"

using namespace Geometry2d;

namespace Planning {

Trajectory PathTargetPlanner::plan(PlanRequest&& request) {
    // Collect obstacles
    Geometry2d::ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    Trajectory ball_trajectory;
    FillObstacles(request, &static_obstacles, &dynamic_obstacles,
                  true, &ball_trajectory);

    // If we start inside of an obstacle, give up and let another planner take
    // care of it.
    if (static_obstacles.hit(request.start.pose.position())) {
        std::cout << "Hit static obstacle" << std::endl;
        reset();
        return Trajectory();
    }

    auto command = std::get<PathTargetCommand>(request.motionCommand);
    RobotInstant goalInstant = command.pathGoal;
    Point goalPoint = goalInstant.pose.position();

    // Debug drawing
    if (request.debug_drawer != nullptr) {
        request.debug_drawer->drawCircle(
            goalPoint, drawRadius, drawColor, drawLayer);
    }

    AngleFunction angle_function = getAngleFunction(request);

    // Call into the sub-object to actually execute the plan.
    Trajectory trajectory = rrt.CreatePlan(
        Replanner::PlanParams{
        request.start,
        goalInstant,
        static_obstacles,
        dynamic_obstacles,
        request.constraints,
        angle_function}, std::move(previous));
    previous = trajectory;
    return trajectory;
}

AngleFunction PathTargetPlanner::getAngleFunction(const PlanRequest& request) {
    std::optional<double> angle_override =
        std::get<PathTargetCommand>(request.motionCommand).angle_override;
    if (angle_override) {
        return AngleFns::faceAngle(*angle_override);
    }
    return AngleFns::tangent;
}


}  // namespace Planning