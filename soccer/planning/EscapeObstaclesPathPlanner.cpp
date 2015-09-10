#include "EscapeObstaclesPathPlanner.hpp"
#include "TrapezoidalPath.hpp"
#include "Tree.hpp"
#include "Util.hpp"

using namespace Geometry2d;

namespace Planning {

std::unique_ptr<Path> EscapeObstaclesPathPlanner::run(
    MotionInstant startInstant, MotionCommand cmd,
    const MotionConstraints& motionConstraints, const ShapeSet* obstacles,
    std::unique_ptr<Path> prevPath) {
    boost::optional<Point> optPrevPt;
    if (prevPath) optPrevPt = prevPath->end().pos;
    const Point unblocked =
        findNonBlockedGoal(startInstant.pos, optPrevPt, obstacles);

    // reuse path if there's not a significantly better spot to target
    if (prevPath && unblocked == prevPath->end().pos) {
        return std::move(prevPath);
    }

    // TODO: build a bezier path instead of a trapezoidal one

    auto path = std::unique_ptr<Path>(
        new TrapezoidalPath(startInstant.pos, motionConstraints.maxSpeed,
                            unblocked, 0, motionConstraints));
    path->setStartTime(timestamp());
    return std::move(path);
}

Point EscapeObstaclesPathPlanner::findNonBlockedGoal(
    Point goal, boost::optional<Point> prevGoal, const ShapeSet* obstacles,
    int maxItr) {
    if (obstacles && obstacles->hit(goal)) {
        FixedStepTree goalTree;
        goalTree.init(goal, obstacles);
        goalTree.step = .1f;  // TODO: config system

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
        // at least one robot radius or the old goal now collides with
        // obstacles.
        float oldDist = (*prevGoal - goal).mag();
        float newDist = (newGoal - goal).mag();
        // TODO: add config value for the threshold, don't use Robot_Radius.
        if (newDist + Robot_Radius < oldDist || obstacles->hit(*prevGoal)) {
            return newGoal;
        } else {
            return *prevGoal;
        }
    }

    return goal;
}

}  // namespace Planning
