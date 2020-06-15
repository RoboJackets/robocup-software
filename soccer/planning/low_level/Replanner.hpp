#pragma once

#include "VelocityProfiling.hpp"
#include "planning/Instant.hpp"
#include "planning/RobotConstraints.hpp"
#include "planning/Trajectory.hpp"
#include "planning/low_level/AnglePlanning.hpp"

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
        const Geometry2d::ShapeSet& static_obstacles;
        const std::vector<DynamicObstacle>& dynamic_obstacles;
        RobotConstraints constraints;
        const AngleFunction& angle_function;
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
    static Trajectory CreatePlan(PlanParams params, Trajectory previous);

    static void createConfiguration(Configuration* cfg);

    /**
     * @brief The threshold by which the goal needs to change before we require
     * a replan.
     */
    static double goalPosChangeThreshold() { return *_goalPosChangeThreshold; }

    /**
     * @brief The duration of the previous path to reuse (from the beginning) in
     * a partial replan. This will be used if possible, although in some cases a
     * full replan will be required.
     */
    static double partialReplanLeadTime() { return *_partialReplanLeadTime; }

private:
    // Attempt a partial replan, and use it only if it is faster.
    static Trajectory checkBetter(const PlanParams& params,
                                  Trajectory previous);

    // Replan part of the trajectory, re-using the first
    // `partialReplanLeadTime()` of the previous trajectory.
    static Trajectory partialReplan(const PlanParams& params,
                                    const Trajectory& previous);

    // Replan from the start without a previous trajectory.
    static Trajectory fullReplan(const PlanParams& params);

    // Whether the trajectory has deviated from the path and requires a replan.
    static bool veeredOffPath(const Trajectory& trajectory, RobotInstant actual,
                              RJ::Time now);

    // Whether the goal has changed past the `goalPosChangeThreshold()`.
    static bool goalChanged(const LinearMotionInstant& prevGoal,
                            const LinearMotionInstant& goal);

    // Get the partial path starting at `now`, plus the partial replan lead time
    // duration.
    static Trajectory partialPath(const Trajectory& prevTrajectory,
                                  RJ::Time now) {
        RJ::Time end_time = now + RJ::Seconds(*_partialReplanLeadTime);
        return prevTrajectory.subTrajectory(prevTrajectory.begin_time(),
                                            end_time);
    }

    static ConfigDouble* _goalPosChangeThreshold;
    static ConfigDouble* _goalVelChangeThreshold;
    static ConfigDouble* _partialReplanLeadTime;

    static constexpr RJ::Seconds _checkBetterDeltaTime = 0.2s;
};

}  // namespace Planning