#include "replanner.hpp"

#include <vector>

#include <rj_constants/constants.hpp>

#include "planning/instant.hpp"
#include "planning/planner/path_planner.hpp"
#include "planning/primitives/angle_planning.hpp"
#include "planning/primitives/create_path.hpp"
#include "planning/trajectory_utils.hpp"
#include "rj_geometry/point.hpp"
#include "rrt_util.hpp"

using namespace rj_geometry;

namespace planning {

void apply_hold(Trajectory* trajectory, std::optional<RJ::Seconds> hold_time) {
    if (hold_time.has_value() && !trajectory->empty() &&
        Twist::nearly_equals(trajectory->last().velocity, Twist::zero())) {
        trajectory->hold_for(hold_time.value());
    }
}

Trajectory Replanner::partial_replan(const PlanParams& params, const Trajectory& previous) {
    std::vector<Point> bias_waypoints;
    for (auto cursor = previous.cursor(params.start.stamp); cursor.has_value();
         cursor.advance(100ms)) {
        bias_waypoints.push_back(cursor.value().position());
    }

    Trajectory pre_trajectory = partial_path(previous, params.start.stamp);
    Trajectory post_trajectory =
        CreatePath::intermediate(pre_trajectory.last().linear_motion(), params.goal, params.constraints.mot,
                        pre_trajectory.end_time(), params.static_obstacles);

    // If we couldn't profile such that velocity at the end of the partial replan period is valid,
    // do a full replan.
    if (post_trajectory.empty() ||
        !Pose::nearly_equals(pre_trajectory.last().pose, post_trajectory.first().pose) ||
        !Twist::nearly_equals(pre_trajectory.last().velocity, post_trajectory.first().velocity)) {
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
        CreatePath::intermediate(params.start.linear_motion(), params.goal, params.constraints.mot,
                        params.start.stamp, params.static_obstacles);

    // if the initial path is empty, the goal must be blocked
    // try to shift the goal_point until it is no longer blocked
    int max_tries = 10;  // try max this many times before giving up (and sending NOP)
    rj_geometry::Point shift_dir = (params.start.position() - params.goal.position).normalized();
    LinearMotionInstant almost_goal = params.goal;
    // start iterating a little bit off of the blocked point
    almost_goal.position += 1.0 * kRobotRadius * shift_dir;
    double shift_size = 1.0 * kRobotRadius;

    for (int i = 0; i < max_tries; i++) {
        if (!path.empty()) {
            break;
        }

        almost_goal.position += shift_dir * shift_size;

        path =
            CreatePath::intermediate(params.start.linear_motion(), almost_goal, params.constraints.mot,
                            params.start.stamp, params.static_obstacles);
    }

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

    return path;
}

Trajectory Replanner::check_better(const Replanner::PlanParams& params, Trajectory previous) {
    Trajectory new_trajectory = partial_replan(params, previous);
    if (!new_trajectory.empty() && new_trajectory.end_time() < previous.end_time()) {
        apply_hold(&new_trajectory, params.hold_time);
        return new_trajectory;
    }

    return previous;
}

Trajectory Replanner::create_plan(Replanner::PlanParams params, Trajectory previous) {
    rj_geometry::Point goal_point = params.goal.position;

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
        if (hit_time - start_time < partial_replan_lead_time() * 2) {
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
        time_remaining > partial_replan_lead_time() * 2) {
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
    return path_error > replanner::PARAM_off_path_threshold;
}

bool Replanner::goal_changed(const LinearMotionInstant& prev_goal,
                             const LinearMotionInstant& goal) {
    double goal_pos_diff = (prev_goal.position - goal.position).mag();
    double goal_vel_diff = (prev_goal.velocity - goal.velocity).mag();
    return goal_pos_diff > goal_pos_change_threshold() ||
           goal_vel_diff > goal_vel_change_threshold();
}

}  // namespace planning
