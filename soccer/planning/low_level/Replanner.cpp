#include "Replanner.hpp"

#include <vector>

#include "RRTUtil.hpp"
#include "planning/Instant.hpp"
#include "planning/TrajectoryUtils.hpp"
#include "planning/low_level/AnglePlanning.hpp"
#include "planning/low_level/CreatePath.hpp"
#include "planning/planner/Planner.hpp"

using namespace Geometry2d;

namespace Planning {

ConfigDouble* Replanner::_goalPosChangeThreshold;
ConfigDouble* Replanner::_goalVelChangeThreshold;
ConfigDouble* Replanner::_partialReplanLeadTime;

REGISTER_CONFIGURABLE(Replanner);

void Replanner::createConfiguration(Configuration* cfg) {
    // NOLINTNEXTLINE
    _goalPosChangeThreshold =
        new ConfigDouble(cfg, "Replanner/goalPosChangeThreshold");
    // NOLINTNEXTLINE
    _goalVelChangeThreshold =
        new ConfigDouble(cfg, "Replanner/goalVelChangeThreshold");
    // NOLINTNEXTLINE
    _partialReplanLeadTime =
        new ConfigDouble(cfg, "Replanner/partialReplanLeadTime");
}

Trajectory Replanner::partialReplan(const PlanParams& params,
                                    const Trajectory& previous) {
    std::vector<Point> biasWaypoints;
    for (auto cursor = previous.cursor(params.start.stamp); cursor.has_value();
         cursor.advance(100ms)) {
        biasWaypoints.push_back(cursor.value().position());
    }

    Trajectory preTrajectory = partialPath(previous, params.start.stamp);
    Trajectory postTrajectory = CreatePath::rrt(
        preTrajectory.last().linear_motion(), params.goal,
        params.constraints.mot, preTrajectory.end_time(),
        params.static_obstacles, params.dynamic_obstacles, biasWaypoints);

    if (postTrajectory.empty()) {
        return fullReplan(params);
    }

    Trajectory combined = Trajectory(std::move(preTrajectory), postTrajectory);

    PlanAngles(&combined, params.start, params.angle_function,
               params.constraints.rot);

    combined.stamp(RJ::now());

    return combined;
}

Trajectory Replanner::fullReplan(const Replanner::PlanParams& params) {
    Trajectory path = CreatePath::rrt(
        params.start.linear_motion(), params.goal,
        params.constraints.mot, params.start.stamp, params.static_obstacles,
        params.dynamic_obstacles);

    if (!path.empty()) {
        if (path.begin_time() > path.end_time()) {
            throw std::runtime_error("Invalid trajectory");
        }

        PlanAngles(&path, params.start, params.angle_function,
                   params.constraints.rot);
    }

    path.stamp(RJ::now());

    if (!path.empty() && !path.angles_valid()) {
        throw std::runtime_error("Path has invalid angles.");
    }

    return std::move(path);
}

Trajectory Replanner::checkBetter(const Replanner::PlanParams& params,
                                  Trajectory previous) {
    Trajectory newTrajectory = partialReplan(params, previous);
    if (!newTrajectory.empty() &&
        newTrajectory.end_time() < previous.end_time()) {
        return std::move(newTrajectory);
    }

    return std::move(previous);
}

Trajectory Replanner::CreatePlan(Replanner::PlanParams params,
                                 Trajectory previous) {
    Geometry2d::Point goalPoint = params.goal.position;

    if (!previous.empty() && !previous.timeCreated().has_value()) {
        throw std::invalid_argument(
            "CreatePlan must be called with a trajectory with a valid creation "
            "time!");
    }

    RJ::Time now = params.start.stamp;

    if (previous.empty() || veeredOffPath(previous, params.start, now) ||
        goalChanged(previous.last().linear_motion(), params.goal)) {
        return fullReplan(params);
    }

    // If we get here, we definitely should have a valid previous trajectory
    // and so it should have a valid creation time (or we would have thrown).
    RJ::Time previous_created_time = previous.timeCreated().value();

    Trajectory previous_trajectory = std::move(previous);

    RJ::Time start_time = std::clamp(now, previous_trajectory.begin_time(),
                                     previous_trajectory.end_time());
    const RJ::Seconds timeRemaining{previous_trajectory.end_time() -
                                    start_time};

    RJ::Time hit_time = RJ::Time::max();

    // Use short-circuiting to only check dynamic trajectories if necessary.
    bool shouldPartialReplan =
        TrajectoryHitsStatic(previous_trajectory, params.static_obstacles,
                             start_time, &hit_time) ||
        TrajectoryHitsDynamic(previous_trajectory, params.dynamic_obstacles,
                              start_time, nullptr, &hit_time);

    if (shouldPartialReplan) {
        if (RJ::Seconds(hit_time - start_time).count() <
            *_partialReplanLeadTime * 2) {
            return fullReplan(params);
        }
        return partialReplan(params, previous_trajectory);
    }

    // Make fine corrections when we are close to the target
    // because the old target might be a bit off
    if (params.start.position().distTo(goalPoint) < Robot_Radius) {
        std::optional<RobotInstant> nowInstant =
            previous_trajectory.evaluate(now);
        if (nowInstant) {
            params.start = *nowInstant;
            return fullReplan(params);
        }
    }

    if (now - previous_created_time > _checkBetterDeltaTime &&
        timeRemaining > RJ::Seconds{*_partialReplanLeadTime} * 2) {
        return checkBetter(params, previous_trajectory);
    }

    previous_trajectory.stamp(RJ::now());
    return previous_trajectory;
}

bool Replanner::veeredOffPath(const Trajectory& trajectory, RobotInstant actual,
                              RJ::Time now) {
    constexpr double kReplanThreshold = 0.5;

    std::optional<RobotInstant> maybe_instant = trajectory.evaluate(now);

    // If we don't have an instant, assume we're past the end of the path.
    if (!maybe_instant.has_value()) {
        maybe_instant = trajectory.last();
    }
    RobotInstant instant = maybe_instant.value();

    double path_error = (instant.position() - actual.position()).mag();
    return path_error > kReplanThreshold;
}

bool Replanner::goalChanged(const LinearMotionInstant& prevGoal,
                            const LinearMotionInstant& goal) {
    double goalPosDiff = (prevGoal.position - goal.position).mag();
    double goalVelDiff = (prevGoal.velocity - goal.velocity).mag();
    return goalPosDiff > *_goalPosChangeThreshold ||
           goalVelDiff > *_goalVelChangeThreshold;
}

}  // namespace Planning
