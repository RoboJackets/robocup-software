#include "TargetVelPathPlanner.hpp"
#include "TrapezoidalPath.hpp"
#include <cmath>

using namespace Geometry2d;

namespace Planning {

bool TargetVelPathPlanner::shouldReplan(
    MotionInstant startInstant, MotionCommand cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, const Path* prevPath) {
    if (SingleRobotPathPlanner::shouldReplan(startInstant, motionConstraints,
                                             obstacles, prevPath)) {
        return true;
    }

    // TODO: check for different target velocity

    // TODO: Check to see if previous obstacles moved out of the way and we can
    // move further than before

    return false;
}

// TODO(justbuchanan): Paths aren't dynamically feasible sometimes because it
// doesn't account for initial velocity
std::unique_ptr<Path> TargetVelPathPlanner::run(
    MotionInstant startInstant, MotionCommand cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, std::unique_ptr<Path> prevPath) {
    assert(cmd.getCommandType() == MotionCommand::WorldVel);

    // reuse the old path if possible
    if (!shouldReplan(startInstant, cmd, motionConstraints, obstacles,
                      prevPath.get())) {
        return std::move(prevPath);
    }

    // TODO: what do if start pos is inside an obstacle?

    Point vel = cmd.getWorldVel();

    const Point dir = vel.normalized();
    const Point start = startInstant.pos;

    float minDist = 0;
    float maxDist =
        sqrtf(powf(Field_Dimensions::Current_Dimensions.FloorLength(), 2) +
              powf(Field_Dimensions::Current_Dimensions.FloorWidth(), 2));

    float nonblockedPathLen = minDist;
    for (float pathLen = minDist; pathLen < maxDist; pathLen += 0.1) {
        Geometry2d::Segment pathSegment(start, start + (pathLen * dir));
        if (obstacles->hit(pathSegment)) break;
        nonblockedPathLen = pathLen;
    }

    MotionConstraints moddedConstraints = motionConstraints;
    moddedConstraints.maxSpeed = cmd.getWorldVel().mag();

    auto path = std::unique_ptr<Path>(new TrapezoidalPath(
        start, startInstant.vel.mag(), start + nonblockedPathLen * dir, 0,
        moddedConstraints));
    path->setStartTime(timestamp());
    return std::move(path);
}

}  // namespace Planning
