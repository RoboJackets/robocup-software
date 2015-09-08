#include "TargetVelPathPlanner.hpp"
#include "TrapezoidalPath.hpp"
#include <cmath>

using namespace Geometry2d;

namespace Planning {

bool TargetVelPathPlanner::shouldReplan(
    MotionInstant startInstant, MotionCommand cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, const Path* prevPath) {
    if (!prevPath || !prevPath->destination()) return true;

    // TODO: refactor the things that are in common with RRTPlanner

    // if this number of microseconds passes since our last path plan, we
    // automatically replan
    const Time kPathExpirationInterval = replanTimeout() * SecsToTimestamp;
    if ((timestamp() - prevPath->startTime()) > kPathExpirationInterval) {
        return true;
    }

    float timeIntoPath =
        ((float)(timestamp() - prevPath->startTime())) * TimestampToSecs +
        1.0f / 60.0f;

    MotionInstant target;
    boost::optional<MotionInstant> optTarget = prevPath->evaluate(timeIntoPath);
    if (optTarget) {
        target = *optTarget;
    } else {
        // We went off the end of the path, so use the end for calculations.
        target = *prevPath->destination();
    }

    float pathError = (target.pos - startInstant.pos).mag();
    float replanThreshold = *motionConstraints._replan_threshold;

    //  invalidate path if current position is more than the
    //  replanThreshold
    if (*motionConstraints._replan_threshold != 0 &&
        pathError > replanThreshold) {
        return true;
    }

    float hitTime = 0;
    if (prevPath->hit(*obstacles, hitTime, timeIntoPath)) {
        return true;
    }

    // // if the destination of the current path is greater than X m away
    // // from the target destination, we invalidate the path. This
    // // situation could arise if the path destination changed.
    // float goalPosDiff = (prevPath->destination()->pos - goal.pos).mag();
    // float goalVelDiff = (prevPath->destination()->vel - goal.vel).mag();
    // if (goalPosDiff > goalChangeThreshold() ||
    //     goalVelDiff > goalChangeThreshold()) {
    //     // FIXME: goalChangeThreshold shouldn't be used for velocities as it
    //     // is above
    //     return true;
    // }


    // TODO: Check to see if previous obstacles moved out of the way and we can move further than before

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
        sqrtf(powf(Field_Dimensions::Current_Dimensions.FloorLength(), 2)+
              powf(Field_Dimensions::Current_Dimensions.FloorWidth(), 2));

    float nonblockedPathLen = minDist;
    for (float pathLen = minDist; pathLen < maxDist; pathLen += 0.1) {
        Geometry2d::Segment pathSegment(start, start + (pathLen * dir));
        if (obstacles->hit(pathSegment)) break;
        nonblockedPathLen = pathLen;
    }


    MotionConstraints moddedConstraints = motionConstraints;
    moddedConstraints.maxSpeed = cmd.getWorldVel().mag();

    auto path =  std::unique_ptr<Path>(new TrapezoidalPath(start, startInstant.vel.mag(),
                    start + nonblockedPathLen*dir, 0,
                    moddedConstraints));
    path->setStartTime(timestamp());
    return std::move(path);
}

}  // namespace Planning
