#pragma once
#include <vector>
#include "Planner.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"

namespace Planning {
class PathTargetPlanner : public PlannerForCommandType<PathTargetCommand> {
public:
    PathTargetPlanner() : PlannerForCommandType("PathTargetPlanner") {}
    ~PathTargetPlanner() override = default;

    Trajectory plan(PlanRequest&& request);

    static void createConfiguration(Configuration* cfg);

    double drawRadius = Robot_Radius;
    QColor drawColor = Qt::black;
    QString drawLayer = "Planning";

    static double goalPosChangeThreshold() { return *_goalPosChangeThreshold; }
    static double partialReplanLeadTime() { return *_partialReplanLeadTime; }

private:
    Trajectory checkBetter(PlanRequest&& request, RobotInstant goalInstant);
    Trajectory partialReplan(PlanRequest&& request, RobotInstant goalInstant);
    Trajectory fullReplan(PlanRequest&& request, RobotInstant goalInstant);
    AngleFunction getAngleFunction(const PlanRequest& request);

    RobotInstant getGoalInstant(const PlanRequest& request) const;
    bool veeredOffPath(const PlanRequest& request) const;
    bool goalChanged(const RobotInstant& prevGoal,
                     const RobotInstant& goal) const;

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
}  // namespace Planning
