#include "TuningPathPlanner.hpp"

#include "planning/MotionCommand.hpp"

namespace Planning {

std::unique_ptr<Path> TuningPathPlanner::run(PlanRequest& planRequest) {
    const MotionInstant& startInstant = planRequest.start;
    const auto& motionConstraints = planRequest.constraints.mot;
    const geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    std::unique_ptr<Path>& prevPath = planRequest.prevPath;

    const Planning::TuningPathCommand& command =
        dynamic_cast<const Planning::TuningPathCommand&>(
            *planRequest.motionCommand);

    if (shouldReplan(planRequest)) {
      geometry2d::Point endTarget = command.pathGoal.pos;
      float endSpeed = command.pathGoal.vel.mag();
      // Tells the robot that is actually in a different location
      // This forces the PID Tuner to kick in to move the robot to its new
      // location
      auto path = std::unique_ptr<Path>(
          new TrapezoidalPath(endTarget, startInstant.vel.mag(), endTarget,
                              endSpeed, motionConstraints));
      path->setStartTime(RJ::now());
      return std::move(path);
    } else {
        return std::move(prevPath);
    }
}

bool TuningPathPlanner::shouldReplan(const PlanRequest& planRequest) const {
    const MotionConstraints& motionConstraints = planRequest.constraints.mot;
    const geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    const Path* prevPath = planRequest.prevPath.get();

    const Planning::TuningPathCommand& command =
        dynamic_cast<const Planning::TuningPathCommand&>(
            *planRequest.motionCommand);

    if (!prevPath) {
        return true;
    } else {
      geometry2d::Point endTarget = command.pathGoal.pos;
      float endSpeed = command.pathGoal.vel.mag();
      float targetPosChange = (prevPath->end().motion.pos - endTarget).mag();
      float targetVelChange = prevPath->end().motion.vel.mag() - endSpeed;

      if (targetPosChange > SingleRobotPathPlanner::goalPosChangeThreshold() ||
          targetVelChange > SingleRobotPathPlanner::goalVelChangeThreshold()) {
        // FIXME: goalChangeThreshold shouldn't be used for checking
        // speed differences as it is in the above 'if' statement
        return true;
        }
    }

    return false;
}

}  // namespace Planning
