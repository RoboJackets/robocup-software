#include "DirectTargetPathPlanner.hpp"
#include "MotionCommand.hpp"

using namespace Geometry2d;
namespace Planning {

double vectorInDirection(Point point, Point direction) {
    const auto vector = point.dot(direction.normalized());
    return std::max(vector, 0.0);
}

std::unique_ptr<Path> DirectTargetPathPlanner::run(PlanRequest& planRequest) {
    const RobotInstant& startInstant = planRequest.start;
    const auto& motionConstraints = planRequest.constraints.mot;
    const Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    std::unique_ptr<Path>& prevPath = planRequest.prevPath;

    const Planning::DirectPathTargetCommand& command =
        dynamic_cast<const Planning::DirectPathTargetCommand&>(
            *planRequest.motionCommand);

    if (shouldReplan(planRequest)) {
        Geometry2d::Point endTarget = command.pathGoal.pos;
        const auto direction = (endTarget - startInstant.motion.pos).normalized();

        float endSpeed = command.pathGoal.vel.mag();
        auto path = std::make_unique<TrapezoidalPath>(
            startInstant.motion.pos, vectorInDirection(startInstant.motion.vel, direction),
            endTarget, vectorInDirection(command.pathGoal.vel, direction),
            motionConstraints);
        path->setStartTime(RJ::now());
        return std::move(path);
    } else {
        return std::move(prevPath);
    }
}

bool DirectTargetPathPlanner::shouldReplan(
    const PlanRequest& planRequest) const {
    const MotionConstraints& motionConstraints = planRequest.constraints.mot;
    const Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    const Path* prevPath = planRequest.prevPath.get();

    const Planning::DirectPathTargetCommand& command =
        dynamic_cast<const Planning::DirectPathTargetCommand&>(
            *planRequest.motionCommand);

    if (!prevPath) {
        return true;
    } else {
        // For DirectTarget commands, we replan if the goal position or velocity
        // have changed beyond a certain threshold
        Geometry2d::Point endTarget = command.pathGoal.pos;
        float endSpeed = command.pathGoal.vel.mag();
        float targetPosChange = (prevPath->end().motion.pos - endTarget).mag();
        float targetVelChange = prevPath->end().motion.vel.mag() - endSpeed;

        if (targetPosChange >
                SingleRobotPathPlanner::goalPosChangeThreshold() ||
            targetVelChange >
                SingleRobotPathPlanner::goalPosChangeThreshold()) {
            // FIXME: goalChangeThreshold shouldn't be used for checking
            // speed differences as it is in the above 'if' statement
            return true;
        }
    }

    return false;
}

}  // namespace Planning
