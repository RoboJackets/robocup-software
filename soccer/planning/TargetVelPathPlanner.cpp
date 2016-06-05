#include "TargetVelPathPlanner.hpp"
#include "TrapezoidalPath.hpp"
#include "EscapeObstaclesPathPlanner.hpp"
#include <Configuration.hpp>
#include <cmath>
#include <boost/range/irange.hpp>

using namespace std;
using namespace Geometry2d;

namespace Planning {

REGISTER_CONFIGURABLE(TargetVelPathPlanner);

ConfigDouble* TargetVelPathPlanner::_targetVelChangeReplanThreshold;

void TargetVelPathPlanner::createConfiguration(Configuration* cfg) {
    _targetVelChangeReplanThreshold = new ConfigDouble(
        cfg, "TargetVelPathPlanner/velChangeReplanThreshold", 0.05,
        "If the target velocity changes by this much (in m/s), the planner is "
        "rerun.  Otherwise the previous path may be kept.");
}

Point TargetVelPathPlanner::calculateNonblockedPathEndpoint(
    Point start, Point dir, const ShapeSet& obstacles) const {
    dir = dir.normalized();

    // TODO(justbuchanan): handle dynamic obstacles (robots)

    float minDist = 0;
    // @maxDist is the diagonal distance across the floor - no robot can travel
    // a straightline distance further than this
    float maxDist =
        sqrtf(powf(Field_Dimensions::Current_Dimensions.FloorLength(), 2) +
              powf(Field_Dimensions::Current_Dimensions.FloorWidth(), 2));

    // We iteratively test different distances from the current point in the
    // direction of the target velocity.  We choose the furthest point away that
    // is non-blocked.  Searching for the furthest non-blocked distance is done
    // through binary search using std::loser_bound.  We use boost ranges to
    // represent a numeric range.  It only works with integers, so we prescale
    // by 100, giving us cm accuracy with the result.
    constexpr float rangeScaleFactor = 100;
    const auto scaledDistRange = boost::irange(
        (int)(minDist * rangeScaleFactor), (int)(maxDist * rangeScaleFactor));
    auto val = std::lower_bound(
        scaledDistRange.begin(), scaledDistRange.end(), obstacles,
        [start, dir, rangeScaleFactor](int scaledDist,
                                       const Geometry2d::ShapeSet& obstacles) {
            Geometry2d::Segment pathSegment(
                start, start + dir * (scaledDist / rangeScaleFactor));
            // Returns true if a path of the given distance doesn't hit
            // obstacles
            return !obstacles.hit(pathSegment);
        });
    const float nonblockedPathLen = *val / rangeScaleFactor;

    return start + dir * nonblockedPathLen;
}

bool TargetVelPathPlanner::shouldReplan(const SinglePlanRequest &planRequest) const {
    const auto currentInstant = planRequest.startInstant;
    const MotionConstraints& motionConstraints = planRequest.robotConstraints.mot;
    const Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    const Path* prevPath = planRequest.prevPath.get();
    // TODO Undo this hack to use TargetVelPlanner to do Pivot
    const WorldVelTargetCommand &command = static_cast<const WorldVelTargetCommand&>(planRequest.cmd);

    if (SingleRobotPathPlanner::shouldReplan(planRequest))
        return true;

    // See if obstacles have changed such that the end point is significantly
    // different
    const Point newEndpoint = calculateNonblockedPathEndpoint(
        prevPath->start().motion.pos, command.worldVel, obstacles);
    const float endChange = (newEndpoint - prevPath->end().motion.pos).mag();
    if (endChange > SingleRobotPathPlanner::goalChangeThreshold()) {
        return true;
    }

    // Replan if the maxSpeed of the previous path differs too much from the
    // command velocity
    if (auto trapezoidalPath = dynamic_cast<const TrapezoidalPath*>(prevPath)) {
        const float velChange =
                command.worldVel.mag() - trapezoidalPath->maxSpeed();
        if (velChange > *_targetVelChangeReplanThreshold) {
            return true;
        }
        return false;
    } else {
        throw std::runtime_error(
            "TargetVelPathPlanner expected a prevPath of type "
            "'TrapezoidalPath'");
    }
}

// TODO(justbuchanan): Paths aren't dynamically feasible sometimes because it
// doesn't account for initial velocity
std::unique_ptr<Path> TargetVelPathPlanner::run(SinglePlanRequest &planRequest) {
    const MotionInstant &startInstant = planRequest.startInstant;
    const MotionCommand &cmd = planRequest.cmd;
    const auto &motionConstraints = planRequest.robotConstraints.mot;
    const Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    std::unique_ptr<Path> &prevPath = planRequest.prevPath;

    // If the start point is in an obstacle, escape from it
    if (obstacles.hit(startInstant.pos)) {
        EscapeObstaclesPathPlanner escapePlanner;
        EmptyCommand emptyCommand;
        return escapePlanner.run(planRequest);
    }

    // TODO Undo this hack to use TargetVelPlanner to do Pivot
    const WorldVelTargetCommand &command = static_cast<const WorldVelTargetCommand&>(cmd);

    if (shouldReplan(planRequest)) {
        // Choose the furthest endpoint we can that doesn't hit obstacles
        Point endpoint = calculateNonblockedPathEndpoint(
            startInstant.pos, command.worldVel, obstacles);
        MotionConstraints moddedConstraints = motionConstraints;
        moddedConstraints.maxSpeed = command.worldVel.mag();

        // Make a path from the start point in the direction of the target vel
        // that ends at the calculated endpoint
        auto path = std::unique_ptr<Path>(
            new TrapezoidalPath(startInstant.pos, startInstant.vel.mag(),
                                endpoint, 0, moddedConstraints));
        path->setStartTime(RJ::timestamp());
        return std::move(path);
    } else {
        return std::move(prevPath);
    }
}

}  // namespace Planning
