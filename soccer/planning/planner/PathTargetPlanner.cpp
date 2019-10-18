#include <rrt/planning/Path.hpp>
#include <planning/trajectory/PathSmoothing.hpp>
#include <planning/trajectory/VelocityProfiling.hpp>
#include "PathTargetPlanner.hpp"

#include "planning/trajectory/RRTUtil.hpp"

namespace Planning {

Trajectory PathTargetPlanner::plan(Planning::PlanRequest &&request) {
    using Geometry2d::Point;
    using Geometry2d::Pose;

    Trajectory result = std::move(request.prevTrajectory);

    RobotInstant start_instant;
    start_instant.pose = request.start.pose;
    start_instant.velocity = request.start.velocity;
    start_instant.stamp = request.start.timestamp;

    if (!result.empty()) {
        auto maybe_start = result.EvaluateTime(RJ::now());
        if (maybe_start) {
            start_instant = *maybe_start;
        }
    }

    PathTargetCommand command = std::get<PathTargetCommand>(request.motionCommand);

    auto state_space = std::make_shared<RoboCupStateSpace>(
            Field_Dimensions::Current_Dimensions, std::move(request.obstacles));
    auto rrt = GenerateRRT(start_instant.pose.position(), command.pathGoal.pose.position(), state_space);

    if (rrt.empty()) {
        return Trajectory({});
    }

    // Make the path smooth
    RRT::SmoothPath(rrt, *state_space);

    BezierPath path(rrt, start_instant.velocity.linear(), command.pathGoal.velocity.linear(), request.constraints.mot);
    result = ProfileVelocity(path,
                             start_instant.velocity.linear().mag(),
                             command.pathGoal.velocity.linear().mag(),
                             request.constraints.mot);

    // Draw the trajectory
    result.draw(&(request.context->debug_drawer));

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
    PlanAngles(result, RobotState{start_instant.pose, start_instant.velocity, start_instant.stamp}, angleFunction, request.constraints.rot);
    return result;
}

} // namespace Planning
