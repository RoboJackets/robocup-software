#include "Replanner.hpp"

#include <vector>

#include "RRTUtil.hpp"
#include "planning/Instant.hpp"
#include "planning/TrajectoryUtils.hpp"
#include "planning/planner/Planner.hpp"
#include "planning/primitives/AnglePlanning.hpp"
#include "planning/primitives/CreatePath.hpp"

using namespace Geometry2d;

namespace Planning {

ConfigDouble* Replanner::goal_pos_change_threshold_config;
ConfigDouble* Replanner::goal_vel_change_threshold_config;
ConfigDouble* Replanner::partial_replan_lead_time_config;
ConfigDouble* Replanner::off_path_error_threshold_config;

REGISTER_CONFIGURABLE(Replanner);

void apply_hold(Trajectory* trajectory, std::optional<RJ::Seconds> hold_time) {
    if (hold_time.has_value() && !trajectory->empty() &&
        Twist::nearly_equals(trajectory->last().velocity, Twist::zero())) {
        trajectory->hold_for(hold_time.value());
    }
}

void Replanner::create_configuration(Configuration* cfg) {
    // NOLINTNEXTLINE
    goal_pos_change_threshold_config =
        new ConfigDouble(cfg, "PathPlanner/Replanner/goalPosChangeThreshold");
    // NOLINTNEXTLINE
    goal_vel_change_threshold_config =
        new ConfigDouble(cfg, "PathPlanner/Replanner/goalVelChangeThreshold");
    // NOLINTNEXTLINE
    partial_replan_lead_time_config =
        new ConfigDouble(cfg, "PathPlanner/Replanner/partialReplanLeadTime");
    // NOLINTNEXTLINE
    off_path_error_threshold_config =
        new ConfigDouble(cfg, "PathPlanner/Replanner/offPathErrorThreshold", 0.5);
}

Trajectory Replanner::partial_replan(const PlanParams& params, const Trajectory& previous) {
    std::vector<Point> bias_waypoints;
    for (auto cursor = previous.cursor(params.start.stamp); cursor.has_value();
         cursor.advance(100ms)) {
        bias_waypoints.push_back(cursor.value().position());
    }

    Trajectory pre_trajectory = partial_path(previous, params.start.stamp);
    Trajectory post_trajectory =
        CreatePath::rrt(pre_trajectory.last().linear_motion(), params.goal, params.constraints.mot,
                        pre_trajectory.end_time(), params.static_obstacles,
                        params.dynamic_obstacles, bias_waypoints);

    if (post_trajectory.empty()) {
        return full_replan(params);
    }

    Trajectory combined = Trajectory(std::move(pre_trajectory), post_trajectory);

    plan_angles(&combined, params.start, params.angle_function, params.constraints.rot);

    combined.stamp(RJ::now());

    apply_hold(&combined, params.hold_time);

    return combined;
}

Trajectory Replanner::full_replan(const Replanner::PlanParams& params) {
    Trajectory path =
        CreatePath::rrt(params.start.linear_motion(), params.goal, params.constraints.mot,
                        params.start.stamp, params.static_obstacles, params.dynamic_obstacles);

    if (!path.empty()) {
        if (path.begin_time() > path.end_time()) {
            throw std::runtime_error("Invalid trajectory");
        }

        plan_angles(&path, params.start, params.angle_function, params.constraints.rot);
    }

    path.stamp(RJ::now());

    if (!path.empty() && !path.angles_valid()) {
        throw std::runtime_error("Path has invalid angles.");
    }

    apply_hold(&path, params.hold_time);

    return std::move(path);
}

Trajectory Replanner::check_better(const Replanner::PlanParams& params, Trajectory previous) {
    Trajectory new_trajectory = partial_replan(params, previous);
    if (!new_trajectory.empty() && new_trajectory.end_time() < previous.end_time()) {
        apply_hold(&new_trajectory, params.hold_time);
        return std::move(new_trajectory);
    }

    return std::move(previous);
}

Trajectory Replanner::create_plan(Replanner::PlanParams params, Trajectory previous) {
    Geometry2d::Point goal_point = params.goal.position;

    if (!previous.empty() && !previous.time_created().has_value()) {
        throw std::invalid_argument(
            "CreatePlan must be called with a trajectory with a valid creation "
            "time!");
    }

    RJ::Time now = params.start.stamp;

    if (previous.empty() || veered_off_path(previous, params.start, now) ||
        goal_changed(previous.last().linear_motion(), params.goal)) {
        return full_replan(params);
    }

    // If we get here, we definitely should have a valid previous trajectory
    // and so it should have a valid creation time (or we would have thrown).
    RJ::Time previous_created_time = previous.time_created().value();

    Trajectory previous_trajectory = std::move(previous);

    RJ::Time start_time =
        std::clamp(now, previous_trajectory.begin_time(), previous_trajectory.end_time());
    const RJ::Seconds time_remaining{previous_trajectory.end_time() - start_time};

    RJ::Time hit_time = RJ::Time::max();

    // Use short-circuiting to only check dynamic trajectories if necessary.
    bool should_partial_replan =
        trajectory_hits_static(previous_trajectory, params.static_obstacles, start_time,
                               &hit_time) ||
        trajectory_hits_dynamic(previous_trajectory, params.dynamic_obstacles, start_time, nullptr,
                                &hit_time);

    if (should_partial_replan) {
        if (RJ::Seconds(hit_time - start_time).count() < *partial_replan_lead_time_config * 2) {
            return full_replan(params);
        }
        return partial_replan(params, previous_trajectory);
    }

    // Make fine corrections when we are close to the target
    // because the old target might be a bit off
    if (params.start.position().dist_to(goal_point) < kRobotRadius) {
        std::optional<RobotInstant> now_instant = previous_trajectory.evaluate(now);
        if (now_instant) {
            params.start = *now_instant;
            return full_replan(params);
        }
    }

    if (now - previous_created_time > kCheckBetterDeltaTime &&
        time_remaining > RJ::Seconds{*partial_replan_lead_time_config} * 2) {
        return check_better(params, previous_trajectory);
    }

    previous_trajectory.stamp(RJ::now());
    return previous_trajectory;
}

bool Replanner::veered_off_path(const Trajectory& trajectory, RobotInstant actual, RJ::Time now) {
    std::optional<RobotInstant> maybe_instant = trajectory.evaluate(now);

    // If we don't have an instant, assume we're past the end of the path.
    if (!maybe_instant.has_value()) {
        maybe_instant = trajectory.last();
    }
    RobotInstant instant = maybe_instant.value();

    double path_error = (instant.position() - actual.position()).mag();
    return path_error > *off_path_error_threshold_config;
}

bool Replanner::goal_changed(const LinearMotionInstant& prev_goal,
                             const LinearMotionInstant& goal) {
    double goal_pos_diff = (prev_goal.position - goal.position).mag();
    double goal_vel_diff = (prev_goal.velocity - goal.velocity).mag();
    return goal_pos_diff > *goal_pos_change_threshold_config ||
           goal_vel_diff > *goal_vel_change_threshold_config;
}

}  // namespace Planning
