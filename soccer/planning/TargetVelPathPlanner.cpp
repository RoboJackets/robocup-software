#include "TargetVelPathPlanner.hpp"

using namespace Geometry2d;

namespace Planning {

std::unique_ptr<Path> TargetVelPathPlanner::run(
    MotionInstant startInstant, MotionCommand cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, std::unique_ptr<Path> prevPath) {
    Point vel = cmd.getWorldVel();

    // TODO: obstacles
    // TODO: trapezoidal motion profile

    // TODO: max velocity

    // TODO: when to replan?

    return nullptr;
}

}  // namespace Planning
