#include "EscapeObstaclesPathPlanner.hpp"

#include <Configuration.hpp>
#include <optional>
#include <vector>

#include "planning/low_level/AnglePlanning.hpp"
#include "planning/low_level/CreatePath.hpp"
#include "planning/low_level/RRTUtil.hpp"
#include "planning/low_level/RoboCupStateSpace.hpp"

using namespace Geometry2d;
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

Trajectory EscapeObstaclesPathPlanner::plan(const PlanRequest& planRequest) {
    const RobotInstant& startInstant = planRequest.start;
    const auto& motionConstraints = planRequest.constraints.mot;

    Geometry2d::ShapeSet obstacles;
    FillObstacles(planRequest, &obstacles, nullptr, false, nullptr);

    if (!obstacles.hit(startInstant.position())) {
        // Stop in place
        Trajectory result{{RobotInstant{startInstant.pose, Twist::Zero(),
                                        startInstant.stamp}}};
        result.mark_angles_valid();
        result.stamp(RJ::now());
        return result;
    }

    std::optional<Point> optPrevPt;
    const Point unblocked =
        findNonBlockedGoal(startInstant.position(), optPrevPt, obstacles, 300);

    LinearMotionInstant goal{unblocked, Point()};
    auto result = CreatePath::simple(startInstant.linear_motion(), goal,
                                     motionConstraints, startInstant.stamp);
    PlanAngles(&result, startInstant, AngleFns::tangent,
               planRequest.constraints.rot);
    result.stamp(RJ::now());
    return std::move(result);
}

Point EscapeObstaclesPathPlanner::findNonBlockedGoal(
    Point goal, std::optional<Point> prevGoal, const ShapeSet& obstacles,
    int maxItr) {
    if (obstacles.hit(goal)) {
        auto stateSpace = std::make_shared<RoboCupStateSpace>(
            Field_Dimensions::Current_Dimensions, obstacles);
        RRT::Tree<Point> rrt(stateSpace, Point::hash, 2);
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
