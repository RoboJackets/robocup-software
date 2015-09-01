#include "SingleRobotPathPlanner.hpp"

namespace Planning {

REGISTER_CONFIGURABLE(SingleRobotPathPlanner);

ConfigDouble* SingleRobotPathPlanner::_goalChangeThreshold;
ConfigDouble* SingleRobotPathPlanner::_replanTimeout;

void SingleRobotPathPlanner::createConfiguration(Configuration* cfg) {
    _replanTimeout = new ConfigDouble(cfg, "PathPlanner/replanTimeout", 5);
    _goalChangeThreshold =
        new ConfigDouble(cfg, "PathPlanner/goalChangeThreshold", 0.025);
}

}  // namespace Planning
