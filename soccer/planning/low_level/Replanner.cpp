#include "Replanner.hpp"

#include <planning/planner/Planner.hpp>
#include <vector>

#include "planning/trajectory/RRTUtil.hpp"

using namespace Geometry2d;

namespace Planning {

ConfigDouble* Replanner::_goalPosChangeThreshold;
ConfigDouble* Replanner::_goalVelChangeThreshold;
ConfigDouble* Replanner::_partialReplanLeadTime;

REGISTER_CONFIGURABLE(Replanner);

void Replanner::createConfiguration(Configuration* cfg) {
    _goalPosChangeThreshold =
        new ConfigDouble(cfg, "Replanner/goalPosChangeThreshold");
    _goalVelChangeThreshold =
        new ConfigDouble(cfg, "Replanner/goalVelChangeThreshold");
    _partialReplanLeadTime =
        new ConfigDouble(cfg, "Replanner/partialReplanLeadTime");
}

Trajectory Replanner::partialReplan(const PlanParams& params,
                                     const Trajectory& previous) {
    std::vector<Point> biasWaypoints;
    for (auto it = previous.iterator(RJ::now(), 100ms);
         (*it).stamp < previous.end_time(); ++it) {
        biasWaypoints.push_back((*it).pose.position());
    }
    Trajectory preTrajectory = partialPath(previous);
    Trajectory postTrajectory = CreatePath::rrt(
        preTrajectory.last(), params.goal, params.constraints.mot,
        params.static_obstacles, params.dynamic_obstacles, biasWaypoints);

    if (postTrajectory.empty()) {
        return fullReplan(params);
    }

    Trajectory combined =
        Trajectory(std::move(preTrajectory), std::move(postTrajectory));

    PlanAngles(combined, combined.first(), params.angle_function,
               params.constraints.rot);
    return combined;
}

Trajectory Replanner::fullReplan(
    const Replanner::PlanParams& params) {
    Trajectory path =
        CreatePath::rrt(params.start, params.goal, params.constraints.mot,
                        params.static_obstacles, params.dynamic_obstacles);

    if (!path.empty()) {
        PlanAngles(path, path.first(), params.angle_function,
                   params.constraints.rot);
    }

    return std::move(path);
}

Trajectory Replanner::checkBetter(
    const Replanner::PlanParams& params, Trajectory previous) {
    Trajectory newTrajectory = partialReplan(params, previous);
    if (newTrajectory.end_time() < previous.end_time()) {
        return std::move(newTrajectory);
    }
    return Planner::reuse(RJ::now(), params.start, std::move(previous));
}

Trajectory Replanner::CreatePlan(Replanner::PlanParams params,
                                  Trajectory previous) {
    Geometry2d::Point goalPoint = params.goal.pose.position();

    RJ::Time now = RJ::now();

    if (previous.empty() || veeredOffPath(previous, params.start, now) ||
        goalChanged(previous.last(), params.goal)) {
        return fullReplan(params);
    }

    Trajectory previous_trajectory = std::move(previous);

    RJ::Seconds timeIntoTrajectory =
        std::clamp(RJ::Seconds{RJ::now() - previous_trajectory.begin_time()},
                   RJ::Seconds{0s}, previous_trajectory.duration());
    const RJ::Seconds timeRemaining =
        previous_trajectory.duration() - timeIntoTrajectory;

    RJ::Seconds invalidTime = RJ::Seconds::max();

    bool shouldPartialReplan =
        previous_trajectory.hit(params.static_obstacles, timeIntoTrajectory,
                           &invalidTime) ||
        previous_trajectory.intersects(params.dynamic_obstacles, RJ::now(), nullptr,
                                  &invalidTime);

    if (shouldPartialReplan) {
        if (invalidTime.count() - timeIntoTrajectory.count() <
            *_partialReplanLeadTime * 2) {
            return fullReplan(params);
        }
        return partialReplan(params, previous_trajectory);
    }

    // make fine corrections when we are close to the target
    // because the old target might be a bit off
    if (params.start.pose.position().distTo(goalPoint) < Robot_Radius) {
        std::optional<RobotInstant> nowInstant =
            previous_trajectory.evaluate(RJ::now());
        if (nowInstant) {
            params.start = *nowInstant;
            return fullReplan(params);
        }
    }
    if (RJ::now() - previous_trajectory.timeCreated() > _checkBetterDeltaTime &&
        timeRemaining > RJ::Seconds{*_partialReplanLeadTime} * 2) {
        return checkBetter(params, previous_trajectory);
    }

    return Planner::reuse(RJ::now(), params.start, previous_trajectory);
}

bool Replanner::veeredOffPath(const Trajectory& trajectory,
                               RobotInstant actual, RJ::Time now) {
    constexpr double kReplanThreshold = 0.5;

    std::optional<RobotInstant> maybe_instant = trajectory.evaluate(now);

    // If we don't have an instant, assume we're past the end of the path.
    if (!maybe_instant.has_value()) {
        maybe_instant = trajectory.last();
    }
    RobotInstant instant = maybe_instant.value();

    double path_error =
        (instant.pose.position() - actual.pose.position()).mag();
    return path_error > kReplanThreshold;
}

bool Replanner::goalChanged(const RobotInstant& prevGoal,
                             const RobotInstant& goal) {
    double goalPosDiff =
        (prevGoal.pose.position() - goal.pose.position()).mag();
    double goalVelDiff =
        (prevGoal.velocity.linear() - goal.velocity.linear()).mag();
    return goalPosDiff > *_goalPosChangeThreshold ||
           goalVelDiff > *_goalVelChangeThreshold;
}

} // namespace Planning
