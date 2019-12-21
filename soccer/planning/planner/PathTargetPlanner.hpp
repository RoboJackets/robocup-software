#pragma once

#include "Planner.hpp"
#include "planning/trajectory/RoboCupStateSpace.hpp"
#include <Geometry2d/Point.hpp>
#include <vector>
#include <memory>

namespace Planning {

class PathTargetPlanner : public PlannerForCommandType<PathTargetCommand> {
public:
    PathTargetPlanner() {}

    Trajectory plan(PlanRequest&& request) override;

    std::string name() const override { return "PathTargetPlanner"; }
    static void createConfiguration(Configuration* cfg);

    bool isApplicable(const MotionCommand& command) const override {
        return std::holds_alternative<PathTargetCommand>(command) || std::holds_alternative<WorldVelTargetCommand>(command);
    }

    static RJ::Seconds getPartialReplanLeadTime() { return RJ::Seconds(*_partialReplanLeadTime);}
private:
    Trajectory partialPath(const Trajectory& prevTrajectory) {
        return prevTrajectory.subTrajectory(0ms, (RJ::now() - prevTrajectory.begin_time()) + getPartialReplanLeadTime());
    }

    Trajectory fullReplan(PlanRequest&& request);
    Trajectory partialReplan(PlanRequest&& request);
    Trajectory reuse(PlanRequest&& request);
    Trajectory checkBetter(PlanRequest&& request);

    bool goalChanged(const RobotInstant& prevGoal, const RobotInstant& goal) const;
    Trajectory planWithoutAngles(PlanRequest&& request);

    static std::vector<RJ::Time> prevTimes;

    static ConfigDouble* _partialReplanLeadTime;
};
} // namespace Planning
