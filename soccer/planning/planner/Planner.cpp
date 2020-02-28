#include "planning/planner/Planner.hpp"
namespace Planning {

    REGISTER_CONFIGURABLE(Planner);

    ConfigDouble* Planner::_goalPosChangeThreshold;
    ConfigDouble* Planner::_goalVelChangeThreshold;
    ConfigDouble* Planner::_replanTimeout;

    void Planner::createConfiguration(Configuration* cfg) {
        _replanTimeout = new ConfigDouble(cfg, "PlannerForCommandType/replanTimeout", 5);
        _goalPosChangeThreshold =
                new ConfigDouble(cfg, "PlannerForCommandType/goalPosChangeThreshold", 0.025);
        _goalVelChangeThreshold =
                new ConfigDouble(cfg, "PlannerForCommandType/goalVelChangeThreshold", 0.025);
    }
}