#include "SingleRobotPathPlanner.hpp"
#include "TargetVelPathPlanner.hpp"
#include "DirectTargetPathPlanner.hpp"
#include "TargetVelPathPlanner.hpp"
#include "EscapeObstaclesPathPlanner.hpp"
#include "RRTPlanner.hpp"
#include "PivotPathPlanner.hpp"

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
        case MotionCommand::DirectPathTarget:
            planner = new DirectTargetPathPlanner();
            break;

        // TODO Undo this hack to use TargetVelPlanner to do Pivot
        case MotionCommand::Pivot:
            planner = new PivotPathPlanner();
            break;
        case MotionCommand::WorldVel:
            planner = new TargetVelPathPlanner();
            break;
        case MotionCommand::None:
            planner = new EscapeObstaclesPathPlanner();
            break;
        default:
            debugThrow("Command not implemented");
            planner = new EscapeObstaclesPathPlanner();
            break;
    }

    return std::unique_ptr<SingleRobotPathPlanner>(planner);
}

boost::optional<std::function<AngleInstant(MotionInstant)>>
angleFunctionForCommandType(const Planning::RotationCommand& command) {
    switch (command.getCommandType()) {
        case RotationCommand::FacePoint: {
            Geometry2d::Point targetPt =
                static_cast<const Planning::FacePointCommand&>(command)
                    .targetPos;
            std::function<AngleInstant(MotionInstant)> function =
                [targetPt](MotionInstant instant) {
                    return AngleInstant(instant.pos.angleTo(targetPt));
                };
            return function;
        }
        case RotationCommand::FaceAngle: {
            float angle = static_cast<const Planning::FaceAngleCommand&>(
                              command).targetAngle;
            std::function<AngleInstant(MotionInstant)> function =
                [angle](MotionInstant instant) { return AngleInstant(angle); };
            return function;
        }
        case RotationCommand::None:
            return boost::none;
        default:
            debugThrow("RotationCommand Not implemented");
            return boost::none;
    }
}

bool SingleRobotPathPlanner::shouldReplan(
    MotionInstant currentInstant, const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, const Path* prevPath) {
    if (!prevPath) return true;

    // if this number of microseconds passes since our last path plan, we
    // automatically replan
    const RJ::Time kPathExpirationInterval = replanTimeout() * SecsToTimestamp;
    if ((RJ::timestamp() - prevPath->startTime()) > kPathExpirationInterval) {
        return true;
    }

    // Evaluate where the path says the robot should be right now
    float timeIntoPath =
        ((float)(RJ::timestamp() - prevPath->startTime())) * TimestampToSecs +
        1.0f / 60.0f;
    boost::optional<RobotInstant> optTarget = prevPath->evaluate(timeIntoPath);
    // If we went off the end of the path, use the end for calculations.
    MotionInstant target =
        optTarget ? optTarget->motion : prevPath->end().motion;

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
