#include <rrt/planning/Path.hpp>
#include <planning/trajectory/PathSmoothing.hpp>
#include <planning/trajectory/VelocityProfiling.hpp>
#include "PathTargetPlanner.hpp"

#include "planning/trajectory/RRTUtil.hpp"

namespace Planning {

Trajectory PathTargetPlanner::plan(Planning::PlanRequest &&request) {
    using Geometry2d::Point;

    PathTargetCommand command = std::get<PathTargetCommand>(request.motionCommand);
    auto state_space = std::make_shared<RoboCupStateSpace>(
            Field_Dimensions::Current_Dimensions, std::move(request.obstacles));
    auto rrt = GenerateRRT(request.start.pose.position(), command.pathGoal.pose.position(), state_space);
    RRT::SmoothPath(rrt, *state_space);
    BezierPath path(rrt, request.start.velocity.linear(), command.pathGoal.velocity.linear(), request.constraints.mot);
    Trajectory result =
            ProfileVelocity(path,
                            request.start.velocity.linear().mag(),
                            command.pathGoal.velocity.linear().mag(),
                            request.constraints.mot);

    std::function<double(Point, Point, double)> angleFunction =
            [](Point pos, Point vel_linear, double angle) -> double {
                // Find the nearest angle either matching velocity or at a 180 degree angle.
                double angle_delta = fixAngleRadians(vel_linear.angle() - angle);
                if (std::abs(angle_delta) < 90) {
                    return angle + angle_delta;
                } else {
                    return fixAngleRadians(angle + angle_delta + M_PI);
                }
            };
    PlanAngles(result, request.start, angleFunction, request.constraints.rot);
    return result;
}

} // namespace Planning
