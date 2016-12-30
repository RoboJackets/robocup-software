#include "EscapeObstaclesPathPlanner.hpp"
#include <Configuration.hpp>
#include "RRTUtil.hpp"
#include "RoboCupStateSpace.hpp"
#include "TrapezoidalPath.hpp"

using namespace Geometry2d;
using namespace std;

namespace Planning {

REGISTER_CONFIGURABLE(EscapeObstaclesPathPlanner);

ConfigDouble* EscapeObstaclesPathPlanner::_stepSize;
ConfigDouble* EscapeObstaclesPathPlanner::_goalChangeThreshold;

void EscapeObstaclesPathPlanner::createConfiguration(Configuration* cfg) {
    _stepSize = new ConfigDouble(
        cfg, "PathPlanner/EscapeObstaclesPathPlanner/stepSize", 0.1);
    _goalChangeThreshold = new ConfigDouble(
        cfg, "EscapeObstaclesPathPlanner/goalChangeThreshold", Robot_Radius);
}

std::unique_ptr<Path> EscapeObstaclesPathPlanner::run(
    PlanRequest& planRequest) {
    const MotionInstant& startInstant = planRequest.start;
    const auto& motionConstraints = planRequest.constraints.mot;
    const Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    std::unique_ptr<Path>& prevPath = planRequest.prevPath;

    boost::optional<Point> optPrevPt;
    if (prevPath) optPrevPt = prevPath->end().motion.pos;
    const Point unblocked = findNonBlockedGoal(
        startInstant.pos, optPrevPt, obstacles, 300,
        [&](const RRT::Tree<Point>& rrt) {
            if (*RRTConfig::EnableRRTDebugDrawing) {
                DrawRRT(rrt, &planRequest.systemState, planRequest.shellID);
            }
        });

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
    path->setStartTime(RJ::now());
    return std::move(path);
}

Point EscapeObstaclesPathPlanner::findNonBlockedGoal(
    Point goal, boost::optional<Point> prevGoal, const ShapeSet& obstacles,
    int maxItr, std::function<void(const RRT::Tree<Point>&)> rrtLogger) {
    if (obstacles.hit(goal)) {
        auto stateSpace = make_shared<RoboCupStateSpace>(
            Field_Dimensions::Current_Dimensions, obstacles);
        RRT::Tree<Point> rrt(stateSpace);
        rrt.setStartState(goal);
        // note: we don't set goal state because we're not looking for a
        // particular point, just something that isn't blocked
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

        if (rrtLogger) rrtLogger(rrt);

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
