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

std::unique_ptr<SingleRobotPathPlanner> PlannerForCommandType(
    MotionCommand::CommandType type) {
    SingleRobotPathPlanner* planner = nullptr;

    switch (type) {
        case MotionCommand::PathTarget:
            planner = new RRTPlanner(250);
            break;
        case MotionCommand::DirectTarget:
            planner = new DirectTargetPathPlanner();
            break;
        case MotionCommand::WorldVel:
            planner = new TargetVelPathPlanner();
            break;
        default:
            break;
    }

    return std::unique_ptr<SingleRobotPathPlanner>(planner);
}

bool SingleRobotPathPlanner::shouldReplan(
    MotionInstant currentInstant, const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, const Path* prevPath) {
    if (!prevPath) return true;

    // if this number of microseconds passes since our last path plan, we
    // automatically replan
    const Time kPathExpirationInterval = replanTimeout() * SecsToTimestamp;
    if ((timestamp() - prevPath->startTime()) > kPathExpirationInterval) {
        return true;
    }

    // Evaluate where the path says the robot should be right now
    float timeIntoPath =
        ((float)(timestamp() - prevPath->startTime())) * TimestampToSecs +
        1.0f / 60.0f;
    boost::optional<MotionInstant> optTarget = prevPath->evaluate(timeIntoPath);
    // If we went off the end of the path, use the end for calculations.
    MotionInstant target = optTarget ? *optTarget : prevPath->end();

    // invalidate path if current position is more than the replanThreshold away
    // from where it's supposed to be right now
    float pathError = (target.pos - currentInstant.pos).mag();
    float replanThreshold = *motionConstraints._replan_threshold;
    if (*motionConstraints._replan_threshold != 0 &&
        pathError > replanThreshold) {
        return true;
    }

    // Replan if we enter new obstacles
    float hitTime = 0;
    if (prevPath->hit(*obstacles, hitTime, timeIntoPath)) {
        return true;
    }

    return false;
}

}  // namespace Planning
