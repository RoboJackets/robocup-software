#include "DirectTargetPathPlanner.hpp"

namespace Planning {

std::unique_ptr<Path> DirectTargetPathPlanner::run(
    MotionInstant startInstant, MotionCommand cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, std::unique_ptr<Path> prevPath) {
    if (shouldReplan(startInstant, cmd, motionConstraints, obstacles,
                     prevPath.get())) {
        Geometry2d::Point endTarget;
        float endSpeed = cmd.getDirectTarget(endTarget);
        auto path =
            std::unique_ptr<Planning::Path>(new Planning::TrapezoidalPath(
                startInstant.pos, startInstant.vel.mag(), endTarget, endSpeed,
                motionConstraints));
        path->setStartTime(timestamp());
        return std::move(path);
    } else {
        return std::move(prevPath);
    }
}

bool DirectTargetPathPlanner::shouldReplan(
    MotionInstant startInstant, MotionCommand cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, const Path* prevPath) const {
    if (!prevPath) {
        return true;
    } else {
        // For DirectTarget commands, we replan if the goal position or velocity
        // have changed beyond a certain threshold
        Geometry2d::Point endTarget;
        float endSpeed = cmd.getDirectTarget(endTarget);
        float targetPosChange = (prevPath->end().pos - endTarget).mag();
        float targetVelChange = prevPath->end().vel.mag() - endSpeed;

        if (targetPosChange >
                Planning::SingleRobotPathPlanner::goalChangeThreshold() ||
            targetVelChange >
                Planning::SingleRobotPathPlanner::goalChangeThreshold()) {
            // FIXME: goalChangeThreshold shouldn't be used for checking
            // speed differences as it is in the above 'if' statement
            return true;
        }
    }

    return false;
}

}  // namespace Planning
