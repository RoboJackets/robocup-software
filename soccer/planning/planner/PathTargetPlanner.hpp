#pragma once
#include <vector>
#include "Planner.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"

namespace Planning {
class PathTargetPlanner: public PlannerForCommandType<PathTargetCommand> {
public:
    PathTargetPlanner(): PlannerForCommandType<PathTargetCommand>("PathTargetPlanner"), drawRadius(Robot_Radius), drawColor(Qt::black), drawLayer("PathTargetPlanner") {}
    ~PathTargetPlanner() override = default;

    Trajectory plan(PlanRequest &&request);

    static void createConfiguration(Configuration* cfg);

    double drawRadius;
    QColor drawColor;
    QString drawLayer;

    static double goalPosChangeThreshold() { return *_goalPosChangeThreshold; }
    static double partialReplanLeadTime() { return *_partialReplanLeadTime; }
private:
    Trajectory checkBetter(PlanRequest&& request, RobotInstant goalInstant);
    Trajectory partialReplan(PlanRequest&& request, RobotInstant goalInstant);
    Trajectory fullReplan(PlanRequest&& request, RobotInstant goalInstant);

    RobotInstant getGoalInstant(const PlanRequest& request) const;
    bool veeredOffPath(const PlanRequest& request) const;
    bool goalChanged(const RobotInstant &prevGoal,
                                        const RobotInstant &goal) const;

    Trajectory partialPath(const Trajectory& prevTrajectory) {
        return prevTrajectory.subTrajectory(0s, (RJ::now() - prevTrajectory.begin_time()) + RJ::Seconds{*_partialReplanLeadTime});
    }

    static ConfigDouble* _goalPosChangeThreshold;
    static ConfigDouble* _goalVelChangeThreshold;
    static ConfigDouble* _partialReplanLeadTime;
};
} // namespace Planning
