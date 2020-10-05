#pragma once

#include "velocity_profiling.hpp"
#include "planning/instant.hpp"
#include "planning/robot_constraints.hpp"
#include "planning/trajectory.hpp"
#include "planning/primitives/angle_planning.hpp"
#include "configuration.hpp"

namespace Planning {

/**
 * @brief Handles the replanning strategy for an RRT plan generator.
 *
 * This is used to avoid constantly replanning.
 */
class Replanner {
public:
    /**
     * @brief Parameters for planning to a point using the replanner.
     */
    struct PlanParams {
        RobotInstant start;
        LinearMotionInstant goal;
        const rj_geometry::ShapeSet& static_obstacles;
        const std::vector<DynamicObstacle>& dynamic_obstacles;
        RobotConstraints constraints;
        const AngleFunction& angle_function;
        std::optional<RJ::Seconds> hold_time = std::nullopt;
    };

    /**
     * @brief Top-level function to create a plan, possibly using part or all of
     * the previous trajectory.
     *
     * @param params The parameters (start and goal) to use to make a plan.
     * @param previous The previous trajectory. Can be moved into this function
     * if it is no longer needed.
     *
     * @return A planned trajectory.
     */
    static Trajectory create_plan(PlanParams params, Trajectory previous);

    static void create_configuration(Configuration* cfg);

    /**
     * @brief The threshold by which the goal needs to change before we require
     * a replan.
     */
    static double goal_pos_change_threshold() { return *goal_pos_change_threshold_config; }

    /**
     * @brief The duration of the previous path to reuse (from the beginning) in
     * a partial replan. This will be used if possible, although in some cases a
     * full replan will be required.
     */
    static double partial_replan_lead_time() { return *partial_replan_lead_time_config; }

private:
    // Attempt a partial replan, and use it only if it is faster.
    static Trajectory check_better(const PlanParams& params,
                                  Trajectory previous);

    // Replan part of the trajectory, re-using the first
    // `partial_replan_lead_time()` of the previous trajectory.
    static Trajectory partial_replan(const PlanParams& params,
                                    const Trajectory& previous);

    // Replan from the start without a previous trajectory.
    static Trajectory full_replan(const PlanParams& params);

    // Whether the trajectory has deviated from the path and requires a replan.
    static bool veered_off_path(const Trajectory& trajectory, RobotInstant actual,
                              RJ::Time now);

    // Whether the goal has changed past the `goal_pos_change_threshold()`.
    static bool goal_changed(const LinearMotionInstant& prev_goal,
                            const LinearMotionInstant& goal);

    // Get the partial path starting at `now`, plus the partial replan lead time
    // duration.
    static Trajectory partial_path(const Trajectory& prev_trajectory,
                                  RJ::Time now) {
        RJ::Time end_time = now + RJ::Seconds(*partial_replan_lead_time_config);
        return prev_trajectory.sub_trajectory(prev_trajectory.begin_time(),
                                            end_time);
    }

    static ConfigDouble* goal_pos_change_threshold_config;
    static ConfigDouble* goal_vel_change_threshold_config;
    static ConfigDouble* partial_replan_lead_time_config;
    static ConfigDouble* off_path_error_threshold_config;

    static constexpr RJ::Seconds kCheckBetterDeltaTime = 0.2s;
};

}  // namespace Planning