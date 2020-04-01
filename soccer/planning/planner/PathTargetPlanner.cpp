#include "planning/planner/PathTargetPlanner.hpp"
#include "EscapeObstaclesPathPlanner.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"

namespace Planning {

ConfigDouble* PathTargetPlanner::_goalPosChangeThreshold;
ConfigDouble* PathTargetPlanner::_goalVelChangeThreshold;
ConfigDouble* PathTargetPlanner::_partialReplanLeadTime;

REGISTER_CONFIGURABLE(PathTargetPlanner);

void PathTargetPlanner::createConfiguration(Configuration* cfg) {
    _goalPosChangeThreshold =
        new ConfigDouble(cfg, "PathTargetPlanner/goalPosChangeThreshold");
    _goalVelChangeThreshold =
        new ConfigDouble(cfg, "PathTargetPlanner/goalVelChangeThreshold");
    _partialReplanLeadTime =
        new ConfigDouble(cfg, "PathTargetPlanner/partialReplanLeadTime");
}
using namespace Geometry2d;
RobotInstant PathTargetPlanner::getGoalInstant(
    const PlanRequest& request) const {
    auto& command = std::get<PathTargetCommand>(request.motionCommand);
    RobotInstant goalInstant = command.pathGoal;
    Point& goalPoint = goalInstant.pose.position();
    std::optional<Point> prevGoal;
    if (!request.prevTrajectory.empty()) {
        prevGoal = request.prevTrajectory.last().pose.position();
    }
    goalPoint = EscapeObstaclesPathPlanner::findNonBlockedGoal(
        goalPoint, prevGoal, request.static_obstacles);
    return goalInstant;
}

Trajectory PathTargetPlanner::plan(PlanRequest&& request) {
    const Trajectory& prevTrajectory = request.prevTrajectory;
    RobotInstant goalInstant = getGoalInstant(request);
    Point& goalPoint = goalInstant.pose.position();
#pragma omp critical(debug_drawer)
    request.context->debug_drawer.drawCircle(goalPoint, drawRadius, drawColor,
                                             drawLayer);
    if (prevTrajectory.empty() || veeredOffPath(request)) {
        return fullReplan(std::move(request), goalInstant);
    }
    RJ::Seconds timeIntoTrajectory =
        std::clamp(RJ::Seconds{RJ::now() - prevTrajectory.begin_time()},
                   RJ::Seconds{0s}, prevTrajectory.duration());
    const RJ::Seconds timeRemaining =
        prevTrajectory.duration() - timeIntoTrajectory;

    RJ::Seconds invalidTime = RJ::Seconds::max();
    bool shouldPartialReplan = false;
    if (prevTrajectory.hit(request.static_obstacles, timeIntoTrajectory,
                           &invalidTime)) {
        shouldPartialReplan = true;
    } else if (prevTrajectory.intersects(request.dynamic_obstacles, RJ::now(),
                                         nullptr, &invalidTime)) {
        shouldPartialReplan = true;
    }
    if (!shouldPartialReplan &&
        goalChanged(prevTrajectory.last(), goalInstant)) {
        invalidTime = prevTrajectory.duration();
        shouldPartialReplan = true;
    }
    if (shouldPartialReplan) {
        if (invalidTime.count() - timeIntoTrajectory.count() <
            *_partialReplanLeadTime * 2) {
            return fullReplan(std::move(request), goalInstant);
        }
        return partialReplan(std::move(request), goalInstant);
    }
    // make fine corrections when we are realy close to the target
    // because the old target might be a bit off
    if (request.start.pose.position().distTo(goalPoint) < Robot_Radius) {
        std::optional<RobotInstant> nowInstant =
            prevTrajectory.evaluate(RJ::now());
        if (nowInstant) {
            request.start = *nowInstant;
            return fullReplan(std::move(request), goalInstant);
        }
    }
    if (RJ::now() - request.prevTrajectory.timeCreated() >
            _checkBetterDeltaTime &&
        timeRemaining > RJ::Seconds{*_partialReplanLeadTime} * 2) {
        return checkBetter(PlanRequest{request}, goalInstant);
    }
    return reuse(std::move(request));
}
AngleFunction PathTargetPlanner::getAngleFunction(const PlanRequest& request) {
    std::optional<double> angle_override =
        request.context->robot_intents[request.shellID].angle_override;
    if (angle_override) {
        return AngleFns::faceAngle(*angle_override);
    }
    return AngleFns::tangent;
}
Trajectory PathTargetPlanner::partialReplan(PlanRequest&& request,
                                            RobotInstant goalInstant) {
    Trajectory& prevTrajectory = request.prevTrajectory;
    AngleFunction angleFunction = getAngleFunction(request);
    std::vector<Point> biasWaypoints;
    for (auto it = prevTrajectory.iterator(RJ::now(), 100ms);
         (*it).stamp < prevTrajectory.end_time(); ++it) {
        biasWaypoints.push_back((*it).pose.position());
    }
    Trajectory preTrajectory = partialPath(prevTrajectory);
    Trajectory postTrajectory = CreatePath::rrt(
        preTrajectory.last(), goalInstant, request.constraints.mot,
        request.static_obstacles, request.dynamic_obstacles, biasWaypoints);
    if (postTrajectory.empty()) {
        return fullReplan(std::move(request), goalInstant);
    }
    Trajectory comboPath =
        Trajectory(std::move(preTrajectory), std::move(postTrajectory));

    PlanAngles(comboPath, comboPath.first(), angleFunction,
               request.constraints.rot);
    return std::move(comboPath);
}

Trajectory PathTargetPlanner::fullReplan(PlanRequest&& request,
                                         RobotInstant goalInstant) {
    AngleFunction angleFunction = getAngleFunction(request);
    Trajectory path =
        CreatePath::rrt(request.start, goalInstant, request.constraints.mot,
                        request.static_obstacles, request.dynamic_obstacles);
    if (path.empty()) {
        return reuse(std::move(request));
    }
    PlanAngles(path, path.first(), angleFunction, request.constraints.rot);
    return std::move(path);
}

Trajectory PathTargetPlanner::checkBetter(PlanRequest&& request,
                                          RobotInstant goalInstant) {
    PlanRequest requestCopy = request;
    Trajectory newTrajectory = partialReplan(std::move(request), goalInstant);
    if (newTrajectory.end_time() < requestCopy.prevTrajectory.end_time()) {
        return std::move(newTrajectory);
    }
    return reuse(std::move(requestCopy));
}

bool PathTargetPlanner::veeredOffPath(const PlanRequest& request) const {
    const auto& currentInstant = request.start;
    const MotionConstraints& motionConstraints = request.constraints.mot;
    const Trajectory& prevTrajectory = request.prevTrajectory;
    if (prevTrajectory.empty()) return false;
    RJ::Seconds timeIntoPath =
        (RJ::now() - prevTrajectory.begin_time()) + RJ::Seconds(1) / 60;
    std::optional<RobotInstant> optTarget =
        prevTrajectory.evaluate(timeIntoPath);
    // If we went off the end of the path, use the end for calculations.
    RobotInstant target = optTarget ? *optTarget : prevTrajectory.last();
    // invalidate path if current position is more than the replanThreshold away
    // from where it's supposed to be right now
    float pathError =
        (target.pose.position() - currentInstant.pose.position()).mag();
    float replanThreshold = *motionConstraints._replan_threshold;
    if (replanThreshold != 0 && pathError > replanThreshold) {
        return true;
    }
    return false;
}

bool PathTargetPlanner::goalChanged(const RobotInstant& prevGoal,
                                    const RobotInstant& goal) const {
    double goalPosDiff =
        (prevGoal.pose.position() - goal.pose.position()).mag();
    double goalVelDiff =
        (prevGoal.velocity.linear() - goal.velocity.linear()).mag();
    return goalPosDiff > *_goalPosChangeThreshold ||
           goalVelDiff > *_goalVelChangeThreshold;
}

}  // namespace Planning