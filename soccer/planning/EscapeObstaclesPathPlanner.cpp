#include "EscapeObstaclesPathPlanner.hpp"
#include "TrapezoidalPath.hpp"
#include "Tree.hpp"
#include "Util.hpp"
#include <Configuration.hpp>

using namespace Geometry2d;

namespace Planning {

REGISTER_CONFIGURABLE(EscapeObstaclesPathPlanner);

ConfigDouble* EscapeObstaclesPathPlanner::_stepSize;
ConfigDouble* EscapeObstaclesPathPlanner::_goalChangeThreshold;

void EscapeObstaclesPathPlanner::createConfiguration(Configuration* cfg) {
    _stepSize =
        new ConfigDouble(cfg, "EscapeObstaclesPathPlanner/stepSize", 0.1);
    _goalChangeThreshold = new ConfigDouble(
        cfg, "EscapeObstaclesPathPlanner/goalChangeThreshold", Robot_Radius);
}

std::unique_ptr<Path> EscapeObstaclesPathPlanner::run(
    MotionInstant startInstant, MotionCommand cmd,
    const MotionConstraints& motionConstraints, const ShapeSet* obstacles,
    std::unique_ptr<Path> prevPath) {
    boost::optional<Point> optPrevPt;
    if (prevPath) optPrevPt = prevPath->end().pos;
    const Point unblocked =
        findNonBlockedGoal(startInstant.pos, optPrevPt, *obstacles);

    // reuse path if there's not a significantly better spot to target
    if (prevPath && unblocked == prevPath->end().pos) {
        return std::move(prevPath);
    }

    // TODO(justbuchanan): build a bezier path instead of a trapezoidal one.  As
    // is, the path isn't dynamically feasible if the robot has any initial
    // velocity.  We could potentially even use a kinodynamic RRT in
    // findNonBlockedGoal() so it takes initial velocity into account

    auto path = std::unique_ptr<Path>(
        new TrapezoidalPath(startInstant.pos, motionConstraints.maxSpeed,
                            unblocked, 0, motionConstraints));
    path->setStartTime(timestamp());
    return std::move(path);
}

Point EscapeObstaclesPathPlanner::findNonBlockedGoal(
    Point goal, boost::optional<Point> prevGoal, const ShapeSet& obstacles,
    int maxItr) {
    if (obstacles.hit(goal)) {
        FixedStepTree goalTree;
        goalTree.init(goal, &obstacles);
        goalTree.step = stepSize();

        // The starting point is in an obstacle, extend the tree until we find
        // an unobstructed point
        Point newGoal;
        for (int i = 0; i < maxItr; ++i) {
            Point r = RandomFieldLocation();

            // extend to a random point
            Tree::Point* newPoint = goalTree.extend(r);

            // if the new point is not blocked, it becomes the new goal
            if (newPoint && newPoint->hit.empty()) {
                newGoal = newPoint->pos;
                break;
            }
        }

        if (!prevGoal) return newGoal;

        // Only use this newly-found point if it's closer to the desired goal by
        // at least a certain threshold or the old goal now collides with
        // obstacles.
        float oldDist = (*prevGoal - goal).mag();
        float newDist = (newGoal - goal).mag();
        if (newDist + *_goalChangeThreshold < oldDist ||
            obstacles.hit(*prevGoal)) {
            return newGoal;
        } else {
            return *prevGoal;
        }
    }

    return goal;
}

}  // namespace Planning
