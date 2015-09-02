#include "SingleRobotPathPlanner.hpp"
#include "DirectTargetPathPlanner.hpp"
#include "TargetVelPathPlanner.hpp"
#include "RRTPlanner.hpp"

namespace Planning {

REGISTER_CONFIGURABLE(SingleRobotPathPlanner);

ConfigDouble* SingleRobotPathPlanner::_goalChangeThreshold;
ConfigDouble* SingleRobotPathPlanner::_replanTimeout;

void SingleRobotPathPlanner::createConfiguration(Configuration* cfg) {
    _replanTimeout = new ConfigDouble(cfg, "PathPlanner/replanTimeout", 5);
    _goalChangeThreshold =
        new ConfigDouble(cfg, "PathPlanner/goalChangeThreshold", 0.025);
}

std::unique_ptr<Planning::SingleRobotPathPlanner> PlannerForCommandType(
    Planning::MotionCommand::CommandType type) {
    Planning::SingleRobotPathPlanner* planner = nullptr;

    switch (type) {
        case Planning::MotionCommand::PathTarget:
            planner = new Planning::RRTPlanner(250);
            break;
        case Planning::MotionCommand::DirectTarget:
            planner = new Planning::DirectTargetPathPlanner();
            break;
        case Planning::MotionCommand::WorldVel:
            planner = new Planning::TargetVelPathPlanner();
            break;
        default:
            break;
    }

    return std::unique_ptr<Planning::SingleRobotPathPlanner>(planner);
}

}  // namespace Planning
