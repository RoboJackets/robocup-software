#pragma once

#include "VelocityProfiling.hpp"
#include "planning/Instant.hpp"
#include "planning/RobotConstraints.hpp"
#include "planning/Trajectory.hpp"

namespace Planning {

class Replanner {
public:
    struct PlanParams {
        RobotInstant start;
        RobotInstant goal;
        const Geometry2d::ShapeSet& static_obstacles;
        const std::vector<DynamicObstacle>& dynamic_obstacles;
        RobotConstraints constraints;
        const AngleFunction& angle_function;
    };

    Trajectory CreatePlan(PlanParams params, Trajectory previous);

    static void createConfiguration(Configuration* cfg);
    static double goalPosChangeThreshold() { return *_goalPosChangeThreshold; }
    static double partialReplanLeadTime() { return *_partialReplanLeadTime; }

private:
    Trajectory checkBetter(const PlanParams& params, Trajectory previous);
    Trajectory partialReplan(const PlanParams& params, const Trajectory& previous);
    static Trajectory fullReplan(const PlanParams& params);

    static bool veeredOffPath(const Trajectory& trajectory,
                              RobotInstant actual, RJ::Time now);
    static bool goalChanged(const RobotInstant& prevGoal,
                            const RobotInstant& goal);

    Trajectory partialPath(const Trajectory& prevTrajectory) {
        return prevTrajectory.subTrajectory(
            0s, (RJ::now() - prevTrajectory.begin_time()) +
                RJ::Seconds{*_partialReplanLeadTime});
    }

    static ConfigDouble* _goalPosChangeThreshold;
    static ConfigDouble* _goalVelChangeThreshold;
    static ConfigDouble* _partialReplanLeadTime;

    static constexpr RJ::Seconds _checkBetterDeltaTime = 0.2s;
};

} // namespace Planning