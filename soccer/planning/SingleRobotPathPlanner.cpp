#include "SingleRobotPathPlanner.hpp"
#include "DirectTargetPathPlanner.hpp"
#include "EscapeObstaclesPathPlanner.hpp"
#include "InterceptPlanner.hpp"
#include "LineKickPlanner.hpp"
#include "TuningPathPlanner.hpp"
//#include "PivotPathPlanner.hpp"
//#include "RRTPlanner.hpp"
#include "CollectPathPlanner.hpp"
#include "PivotPathPlanner.hpp"
#include "RRTPlanner.hpp"
#include "SettlePathPlanner.hpp"
#include "TargetVelPathPlanner.hpp"

using namespace std;

namespace Planning {

REGISTER_CONFIGURABLE(SingleRobotPathPlanner);

ConfigDouble* SingleRobotPathPlanner::_goalPosChangeThreshold;
ConfigDouble* SingleRobotPathPlanner::_goalVelChangeThreshold;
ConfigDouble* SingleRobotPathPlanner::_replanTimeout;

void SingleRobotPathPlanner::createConfiguration(Configuration* cfg) {
    _replanTimeout = new ConfigDouble(cfg, "PathPlanner/replanTimeout", 5);
    _goalPosChangeThreshold =
        new ConfigDouble(cfg, "PathPlanner/goalPosChangeThreshold", 0.025);
    _goalVelChangeThreshold =
        new ConfigDouble(cfg, "PathPlanner/goalVelChangeThreshold", 0.025);
}

std::unique_ptr<SingleRobotPathPlanner> PlannerForCommandType(
    MotionCommand::CommandType type) {
    SingleRobotPathPlanner* planner = nullptr;
    switch (type) {
        case MotionCommand::PathTarget:
            planner = new RRTPlanner(100, 250);
            break;
        case MotionCommand::DirectPathTarget:
            planner = new DirectTargetPathPlanner();
            break;
        case MotionCommand::TuningPath:
            planner = new TuningPathPlanner();
            break;
        case MotionCommand::Pivot:
            planner = new PivotPathPlanner();
            break;
        case MotionCommand::WorldVel:
            planner = new TargetVelPathPlanner();
            break;
        case MotionCommand::Settle:
            planner = new SettlePathPlanner();
            break;
        case MotionCommand::Collect:
            planner = new CollectPathPlanner();
            break;
        case MotionCommand::LineKick:
            planner = new LineKickPlanner();
            break;
        case MotionCommand::Intercept:
            planner = new InterceptPlanner();
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

void SingleRobotPathPlanner::allDynamicToStatic(
    Geometry2d::ShapeSet& obstacles,
    const std::vector<DynamicObstacle>& dynamicObstacles) {
    for (auto& dynObs : dynamicObstacles) {
        obstacles.add(dynObs.getStaticObstacle());
    }
}

void SingleRobotPathPlanner::splitDynamic(
    Geometry2d::ShapeSet& obstacles, std::vector<DynamicObstacle>& dynamicOut,
    const std::vector<DynamicObstacle>& dynamicObstacles) {
    for (auto& dynObs : dynamicObstacles) {
        if (dynObs.hasPath()) {
            dynamicOut.push_back(dynObs);
        } else {
            obstacles.add(dynObs.getStaticObstacle());
        }
    }
}

AngleFunction angleFunctionForCommandType(const Planning::RotationCommand& command) {
    switch (command.getCommandType()) {
        case RotationCommand::FacePoint: {
            Geometry2d::Point targetPt =
                static_cast<const Planning::FacePointCommand&>(command)
                    .targetPos;
            AngleFunction function =
                [targetPt](MotionInstant instant) {
                    return instant.pos.angleTo(targetPt);
                };
            return function;
        }
        case RotationCommand::FaceAngle: {
            float angle = static_cast<const Planning::FaceAngleCommand&>(
                              command).targetAngle;
            AngleFunction function =
                [angle](MotionInstant instant) { return angle; };
            return function;
        }
        case RotationCommand::None:
            // Return a default angle command to face in the direction of velocity.
            // TODO(Kyle): This can't handle backwards motion. Fix it to target
            // forwards or backwards, whichever is closest.
            return [](MotionInstant instant) {
                double angle = instant.vel.angle();
                return angle;
            };
        default:
            debugThrow("RotationCommand Not implemented");
            return [](MotionInstant instant) {
                double angle = instant.vel.angle();
                return angle;
            };
    }
}

bool SingleRobotPathPlanner::shouldReplan(const PlanRequest& planRequest) {
    const auto currentInstant = planRequest.start;
    const MotionConstraints& motionConstraints = planRequest.constraints.mot;
    const Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    const Path* prevPath = planRequest.prevPath.get();

    if (!prevPath) return true;

    // if this number of microseconds passes since our last path plan, we
    // automatically replan
    // const RJ::Seconds kPathExpirationInterval = RJ::Seconds(replanTimeout());
    // if ((RJ::now() - prevPath->startTime()) > kPathExpirationInterval) {
    //    return true;
    //}

    // Evaluate where the path says the robot should be right now
    RJ::Seconds timeIntoPath =
        (RJ::now() - prevPath->startTime()) + RJ::Seconds(1) / 60;

    std::optional<RobotInstant> optTarget = prevPath->evaluate(timeIntoPath);
    // If we went off the end of the path, use the end for calculations.
    MotionInstant target =
        optTarget ? optTarget->motion : prevPath->end().motion;

    // invalidate path if current position is more than the replanThreshold away
    // from where it's supposed to be right now
    float pathError = (target.pos - currentInstant.motion.pos).mag();
    float replanThreshold = *motionConstraints._replan_threshold;
    if (*motionConstraints._replan_threshold != 0 &&
        pathError > replanThreshold) {
        return true;
    }

    // Replan if we enter new obstacles
    RJ::Seconds hitTime;
    if (prevPath->hit(obstacles, timeIntoPath, &hitTime)) {
        return true;
    }

    return false;
}
}  // namespace Planning
