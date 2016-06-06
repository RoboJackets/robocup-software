#include "LineKickPlanner.hpp"
#include "EscapeObstaclesPathPlanner.hpp"
#include <Configuration.hpp>
#include "RRTPlanner.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

bool LineKickPlanner::shouldReplan(
    const SinglePlanRequest& planRequest) const {
    const MotionConstraints& motionConstraints =
        planRequest.robotConstraints.mot;
    const Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    const Path* prevPath = planRequest.prevPath.get();

    const auto& command = dynamic_cast<const PivotCommand&>(planRequest.cmd);

    if (!prevPath) {
        return true;
    } else {
        // TODO ashaw37: make this better
        float radius = command.radius;
        auto pivotPoint = command.pivotPoint;
        auto pivotTarget = command.pivotTarget;
        auto endTarget =
            pivotPoint + (pivotPoint - pivotTarget).normalized(radius);
        float targetChange = (prevPath->end().motion.pos - endTarget).mag();

        if (targetChange > SingleRobotPathPlanner::goalChangeThreshold()) {
            // return true;
        }
    }
    return false;
}

std::unique_ptr<Path> LineKickPlanner::run(SinglePlanRequest& planRequest) {
    const MotionInstant& startInstant = planRequest.startInstant;
    const auto& motionConstraints = planRequest.robotConstraints.mot;
    const auto& rotationConstraints = planRequest.robotConstraints.rot;
    const Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    std::unique_ptr<Path>& prevPath = planRequest.prevPath;

    const auto& command = dynamic_cast<const PivotCommand&>(planRequest.cmd);

    if (shouldReplan(planRequest)) {
        return nullptr;
    } else {
        return std::move(prevPath);
    }
}
}
