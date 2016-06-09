#include "EscapeObstaclesPathPlanner.hpp"
#include <Configuration.hpp>
#include <rrt/Tree.hpp>
#include "RoboCupStateSpace.hpp"
#include "TrapezoidalPath.hpp"
#include "Util.hpp"

using namespace Geometry2d;
using namespace std;

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
    SinglePlanRequest& planRequest) {
    const MotionInstant& startInstant = planRequest.startInstant;
    const auto& motionConstraints = planRequest.robotConstraints.mot;
    const Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    std::unique_ptr<Path>& prevPath = planRequest.prevPath;

    boost::optional<Point> optPrevPt;
    if (prevPath) optPrevPt = prevPath->end().motion.pos;
    const Point unblocked =
        findNonBlockedGoal(startInstant.pos, optPrevPt, obstacles);

    // reuse path if there's not a significantly better spot to target
    if (prevPath && unblocked == prevPath->end().motion.pos) {
        return std::move(prevPath);
    }

    // TODO(justbuchanan): build a bezier path instead of a trapezoidal one.  As
    // is, the path isn't dynamically feasible if the robot has any initial
    // velocity.  We could potentially even use a kinodynamic RRT in
    // findNonBlockedGoal() so it takes initial velocity into account

    auto path = std::unique_ptr<Path>(
        new TrapezoidalPath(startInstant.pos, motionConstraints.maxSpeed,
                            unblocked, 0, motionConstraints));
    path->setStartTime(RJ::timestamp());
    return std::move(path);
}

Point EscapeObstaclesPathPlanner::findNonBlockedGoal(
    Point goal, boost::optional<Point> prevGoal, const ShapeSet& obstacles,
    int maxItr) {
    if (obstacles.hit(goal)) {
        auto stateSpace = make_shared<RoboCupStateSpace>(
            Field_Dimensions::Current_Dimensions);
        // TODO(justin): set obstacles
        RRT::Tree<Geometry2d::Point> rrt(stateSpace);
        rrt.setStartState(goal);
        // TODO: rrt.setGoalState();
        rrt.setStepSize(stepSize());

        // The starting point is in an obstacle, extend the tree until we find
        // an unobstructed point
        Point newGoal;
        for (int i = 0; i < maxItr; ++i) {
            // extend towards a random point
            RRT::Node<Point>* newNode = rrt.grow();

            // if the new point is not blocked, it becomes the new goal
            if (newNode && !obstacles.hit(newNode->state())) {
                newGoal = newNode->state();
                break;
            }
        }

        if (!prevGoal || obstacles.hit(*prevGoal)) return newGoal;

        // Only use this newly-found point if it's closer to the desired goal by
        // at least a certain threshold
        float oldDist = (*prevGoal - goal).mag();
        float newDist = (newGoal - goal).mag();
        if (newDist + *_goalChangeThreshold < oldDist) {
            return newGoal;
        } else {
            return *prevGoal;
        }
    }

    return goal;
}

}  // namespace Planning
