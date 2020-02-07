#pragma once

#include "Planner.hpp"
#include "planning/trajectory/RoboCupStateSpace.hpp"
#include <Geometry2d/Point.hpp>
#include <vector>
#include <memory>
#include "planning/trajectory/VelocityProfiling.hpp"

namespace Planning {

class PathTargetPlanner : public PlannerForCommandType<PathTargetCommand> {
public:
    PathTargetPlanner(): prevTimes(Num_Shells, RJ::now()-60s), anglePlanningEnabled(true) {}

    Trajectory plan(PlanRequest&& request) override;
    Trajectory planWithoutAngles(PlanRequest&& request) {
        anglePlanningEnabled = false;
        Trajectory path = plan(std::move(request));
        anglePlanningEnabled = true;
        return std::move(path);
    }

    std::string name() const override { return "PathTargetPlanner"; }
    static void createConfiguration(Configuration* cfg);

    bool isApplicable(const MotionCommand& command) const override {
        return std::holds_alternative<PathTargetCommand>(command) || std::holds_alternative<WorldVelTargetCommand>(command);
    }

    static RJ::Seconds getPartialReplanLeadTime() { return RJ::Seconds(*_partialReplanLeadTime);}
private:
    Trajectory partialPath(const Trajectory& prevTrajectory) {
        //todo(Ethan) cut out old parts of the path?
        return prevTrajectory.subTrajectory(0s, (RJ::now() - prevTrajectory.begin_time()) + getPartialReplanLeadTime());
    }

    Trajectory fullReplan(PlanRequest&& request, AngleFunction angleFunction);
    Trajectory partialReplan(PlanRequest&& request, AngleFunction angleFunction);
    Trajectory checkBetter(PlanRequest&& request, AngleFunction angleFunction);

    bool goalChanged(const RobotInstant& prevGoal, const RobotInstant& goal) const;

    //todo(Ethan) delete these probably
    std::vector<RJ::Time> prevTimes;
    bool anglePlanningEnabled;

    static ConfigDouble* _partialReplanLeadTime;
};
} // namespace Planning
