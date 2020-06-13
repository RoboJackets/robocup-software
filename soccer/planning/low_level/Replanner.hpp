#pragma once

#include "VelocityProfiling.hpp"
#include "planning/Instant.hpp"
#include "planning/RobotConstraints.hpp"
#include "planning/Trajectory.hpp"
#include "planning/low_level/AnglePlanning.hpp"

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
    Trajectory partialReplan(const PlanParams& params,
                             const Trajectory& previous);
    static Trajectory fullReplan(const PlanParams& params);

    static bool veeredOffPath(const Trajectory& trajectory, RobotInstant actual,
                              RJ::Time now);
    static bool goalChanged(const RobotInstant& prevGoal,
                            const RobotInstant& goal);

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