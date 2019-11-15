#include <rrt/planning/Path.hpp>
#include <planning/trajectory/PathSmoothing.hpp>
#include <planning/trajectory/VelocityProfiling.hpp>
#include "PathTargetPlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include <Geometry2d/Pose.hpp>
#include <vector>

namespace Planning {

REGISTER_CONFIGURABLE(PathTargetPlanner);

ConfigDouble* PathTargetPlanner::_partialReplanLeadTime;

void PathTargetPlanner::createConfiguration(Configuration* cfg) {
    _partialReplanLeadTime = new ConfigDouble(
            cfg, "PathTargetPlanner/partialReplanLeadTime", 0.2, "partialReplanLeadTime");
}

Trajectory PathTargetPlanner::plan(Planning::PlanRequest &&request) {
    if(!std::holds_alternative<PathTargetCommand>(request.motionCommand)) {
        throw std::invalid_argument("Error in PathTargetPlanner: invalid motionCommand; must be a PathTargetCommand.");
    }

    ++counter;
    using Geometry2d::Point;
    using Geometry2d::Pose;
    using Geometry2d::Twist;

    // make these calls before we move anything out of the request
    //path is completely invalid
    bool isFullReplan = shouldReplan(request);
    // path was interupted by an obstacle
    // or goal has changed
    auto invalidTime = findInvalidTime(request);
    bool isPartialReplan = invalidTime.has_value();
    //this is forces a replan every ~10 iterations <-- todo(Ethan) make sure that's accurate
    bool checkBetter = false;

    Trajectory result = std::move(request.prevTrajectory);

    RobotInstant current_instant;
    current_instant.pose = request.start.pose;
    current_instant.velocity = request.start.velocity;
    current_instant.stamp = request.start.timestamp;

    //assumes the robot is on the path
//    if (!result.empty()) {
//        auto maybe_start = result.evaluate(RJ::now());
//        if (maybe_start) {
//            current_instant = *maybe_start;
//        }
//    }
    PathTargetCommand command = std::get<PathTargetCommand>(request.motionCommand);


    auto state_space = std::make_shared<RoboCupStateSpace>(
            Field_Dimensions::Current_Dimensions, std::move(request.obstacles));

    // Simple case: no path
    const Pose &current_pose = current_instant.pose;
    const Pose &goal_pose = command.pathGoal.pose;
    if (current_pose.position() == goal_pose.position()) {
        std::vector<RobotInstant> instants;
        instants.push_back(RobotInstant(current_pose, Twist(), RJ::now()));
        result = std::move(Trajectory(std::move(instants)));
        //todo(Ethan) fix this
//        result.setDebugText("Invalid Basic Path");
        std::cout << "invalid basic path " << std::endl;
        return std::move(result);
    }

    //todo(Ethan) implement EscapeObstaclePlanner and invoke it here

    RJ::Seconds timeIntoTrajectory = RJ::now() - result.begin_time();
    const RJ::Seconds partialReplanTime(*_partialReplanLeadTime);
    if (invalidTime) {
        const auto timeFromInvalid = *invalidTime - timeIntoTrajectory;
        if (timeFromInvalid < partialReplanTime * 2) {
            isFullReplan = true;
        }
    }
    //todo(Ethan) **maybe** delete checkBetter, it's pointless maybe
    if (counter >= 10) {
        counter = 0;
        if (result.duration() - timeIntoTrajectory > partialReplanTime * 2) {
            checkBetter = true;
        }
    }
    if(isFullReplan) {
        std::vector<Point> new_points = GenerateRRT(current_pose.position(), goal_pose.position(), state_space);
        if (new_points.empty()) {
            std::cout << "RRT failed (full) " << std::endl;
            return Trajectory({});
        }
        RRT::SmoothPath(new_points, *state_space);
        BezierPath new_bezier(new_points, current_instant.velocity.linear(), command.pathGoal.velocity.linear(),
                              request.constraints.mot);
        result = ProfileVelocity(new_bezier,
             current_instant.velocity.linear().mag(),
             command.pathGoal.velocity.linear().mag(),
             request.constraints.mot);
    } else if(isPartialReplan) {
        Trajectory pre_trajectory = result.subTrajectory(0ms, timeIntoTrajectory + partialReplanTime);
        const RobotInstant &middle_instant = pre_trajectory.last();
        std::vector<Point> biasWaypoints;
        biasWaypoints.push_back(current_instant.pose.position() + current_instant.velocity.linear() * 0.2);
        for (auto it = result.iterator(RJ::now(), 100ms); *it != result.last(); ++it) {
            biasWaypoints.push_back((*it).pose.position());
        }
        //todo(Ethan) handle RRT, Bezier, etc. errors generating the path
        std::vector<Point> new_points = GenerateRRT(middle_instant.pose.position(), goal_pose.position(), state_space,
                biasWaypoints);
        if (new_points.empty()) {
            std::cout << "RRT failed (partial) " << std::endl;
            return Trajectory({});
        }
        RRT::SmoothPath(new_points, *state_space);
        BezierPath new_bezier(new_points, current_instant.velocity.linear(), command.pathGoal.velocity.linear(),
                request.constraints.mot);
        Trajectory post_trajectory = ProfileVelocity(new_bezier,
                current_instant.velocity.linear().mag(),
                command.pathGoal.velocity.linear().mag(),
                request.constraints.mot);
        result = Trajectory(pre_trajectory, post_trajectory);
    }
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
    PlanAngles(result, RobotState{current_instant.pose, current_instant.velocity, current_instant.stamp}, angleFunction, request.constraints.rot);
    return std::move(result);
}

bool PathTargetPlanner::shouldReplan(const PlanRequest& request) const {
    if(Planner::shouldReplan(request)) {
        return true;
    }
    return false;
}
std::optional<RJ::Seconds> PathTargetPlanner::findInvalidTime(const PlanRequest& request) const {
    if(request.prevTrajectory.empty()) {
        return std::nullopt;
    }
    const auto timeIntoPrevPath = RJ::now() - request.prevTrajectory.begin_time();
    RJ::Seconds invalidTime;
    if (request.prevTrajectory.hit(request.obstacles, timeIntoPrevPath, &invalidTime)) {
        return invalidTime;
    }
    PathTargetCommand command = std::get<PathTargetCommand>(request.motionCommand);

    //todo(Ethan) check dynamic obstacles
    const RobotInstant& last = request.prevTrajectory.last();
    const RobotInstant& goal = command.pathGoal;
    double goalPosDiff = (last.pose.position() - goal.pose.position()).mag();
    double goalVelDiff = (last.velocity.linear() - goal.velocity.linear()).mag();
    if (goalPosDiff > Planner::goalPosChangeThreshold() || goalVelDiff > Planner::goalVelChangeThreshold()) {
        return request.prevTrajectory.duration();
    }
    return std::nullopt;
}

} // namespace Planning
