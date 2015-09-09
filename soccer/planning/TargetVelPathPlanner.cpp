#include "TargetVelPathPlanner.hpp"
#include "TrapezoidalPath.hpp"
#include <Configuration.hpp>
#include <cmath>

using namespace Geometry2d;

namespace Planning {

REGISTER_CONFIGURABLE(TargetVelPathPlanner);

ConfigDouble* TargetVelPathPlanner::_targetVelChangeReplanThreshold;

void TargetVelPathPlanner::createConfiguration(Configuration* cfg) {
    _targetVelChangeReplanThreshold =
        new ConfigDouble(cfg, "TargetVelPathPlanner/replanThreshold", 0.05);
}

Point TargetVelPathPlanner::calculateNonblockedPathEndpoint(
    Point start, Point dir, const Geometry2d::ShapeSet* obstacles) {
    dir = dir.normalized();

    // TODO(justbuchanan): handle dynamic obstacles (robots)

    float minDist = 0;
    // @maxDist is the diagonal distance across the floor - no robot can travel
    // a straightline distance further than this
    float maxDist =
        sqrtf(powf(Field_Dimensions::Current_Dimensions.FloorLength(), 2) +
              powf(Field_Dimensions::Current_Dimensions.FloorWidth(), 2));

    // TODO: do binary search instead of linear and use a greater resolution
    // than 10cm
    //
    // We iteratively test different distances from the current point in the
    // direction of the target velocity.  We choose the furthest point away that
    // is non-blocked.
    float nonblockedPathLen = minDist;
    for (float pathLen = minDist; pathLen < maxDist; pathLen += 0.1) {
        Geometry2d::Segment pathSegment(start, start + (pathLen * dir));
        if (obstacles->hit(pathSegment)) break;
        nonblockedPathLen = pathLen;
    }
    return start + dir * nonblockedPathLen;
}

bool TargetVelPathPlanner::shouldReplan(
    MotionInstant startInstant, MotionCommand cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, const Path* prevPath) {
    if (SingleRobotPathPlanner::shouldReplan(startInstant, motionConstraints,
                                             obstacles, prevPath))
        return true;

    // See if obstacles have changed such that the end point is significantly
    // different
    const Point newEndpoint = calculateNonblockedPathEndpoint(
        prevPath->start().pos, cmd.getWorldVel(), obstacles);
    const float endChange = (newEndpoint - prevPath->end().pos).mag();
    if (endChange > SingleRobotPathPlanner::goalChangeThreshold()) {
        return true;
    }

    // Replan if the maxSpeed of the previous path differs too much from the
    // command velocity
    auto trapezoidalPath = dynamic_cast<const TrapezoidalPath*>(prevPath);
    if (!trapezoidalPath) {
        throw std::runtime_error(
            "TargetVelPathPlanner expected a prevPath of type "
            "'TrapezoidalPath'");
    }
    const float velChange = cmd.getWorldVel().mag() - trapezoidalPath->maxSpeed();
    const float threshold =
        0.1;  // TODO: config value?  time-dependent threshold?
    if (velChange > threshold) {
        return true;
    }

    return false;
}

// TODO(justbuchanan): Paths aren't dynamically feasible sometimes because it
// doesn't account for initial velocity
std::unique_ptr<Path> TargetVelPathPlanner::run(
    MotionInstant startInstant, MotionCommand cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, std::unique_ptr<Path> prevPath) {
    assert(cmd.getCommandType() == MotionCommand::WorldVel);

    if (obstacles->hit(startInstant.pos)) {
        // TODO: what do if start pos is inside an obstacle?
        // Compare the time to get OUT of an obstacle with obeying velocity to
        // an EscapeObstaclesPlanner path.
    }

    if (shouldReplan(startInstant, cmd, motionConstraints, obstacles,
                     prevPath.get())) {
        Point endpoint = calculateNonblockedPathEndpoint(
            startInstant.pos, cmd.getWorldVel(), obstacles);
        MotionConstraints moddedConstraints = motionConstraints;
        moddedConstraints.maxSpeed = cmd.getWorldVel().mag();

        auto path = std::unique_ptr<Path>(new TrapezoidalPath(
            startInstant.pos, startInstant.vel.mag(), endpoint, 0, moddedConstraints));
        path->setStartTime(timestamp());
        return std::move(path);
    } else {
        return std::move(prevPath);
    }
}

}  // namespace Planning
