#include "DirectTargetPathPlanner.hpp"
#include "MotionCommand.hpp"

namespace Planning {

std::unique_ptr<Path> DirectTargetPathPlanner::run(
    MotionInstant startInstant, const MotionCommand* cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, std::unique_ptr<Path> prevPath) {
    assert(cmd->getCommandType() == Planning::MotionCommand::DirectPathTarget);
    Planning::DirectPathTargetCommand command =
        *static_cast<const Planning::DirectPathTargetCommand*>(cmd);

    if (shouldReplan(startInstant, cmd, motionConstraints, obstacles,
                     prevPath.get())) {
        Geometry2d::Point endTarget = command.pathGoal.pos;
        float endSpeed = command.pathGoal.vel.mag();
        auto path = std::unique_ptr<Path>(
            new TrapezoidalPath(startInstant.pos, startInstant.vel.mag(),
                                endTarget, endSpeed, motionConstraints));
        path->setStartTime(RJ::timestamp());
        return std::move(path);
    } else {
        return std::move(prevPath);
    }
}

bool DirectTargetPathPlanner::shouldReplan(
    MotionInstant startInstant, const MotionCommand* cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, const Path* prevPath) const {
    assert(cmd->getCommandType() == Planning::MotionCommand::DirectPathTarget);
    Planning::DirectPathTargetCommand command =
        *static_cast<const Planning::DirectPathTargetCommand*>(cmd);

    if (!prevPath) {
        return true;
    } else {
        // For DirectTarget commands, we replan if the goal position or velocity
        // have changed beyond a certain threshold
        Geometry2d::Point endTarget = command.pathGoal.pos;
        float endSpeed = command.pathGoal.vel.mag();
        float targetPosChange = (prevPath->end().pos - endTarget).mag();
        float targetVelChange = prevPath->end().vel.mag() - endSpeed;

        if (targetPosChange > SingleRobotPathPlanner::goalChangeThreshold() ||
            targetVelChange > SingleRobotPathPlanner::goalChangeThreshold()) {
            // FIXME: goalChangeThreshold shouldn't be used for checking
            // speed differences as it is in the above 'if' statement
            return true;
        }
    }

    return false;
}

}  // namespace Planning
