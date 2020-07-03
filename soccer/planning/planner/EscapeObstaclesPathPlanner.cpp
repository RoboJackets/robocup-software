#include "EscapeObstaclesPathPlanner.hpp"
#include <easy/profiler.h>

#include <Configuration.hpp>
#include <optional>
#include <vector>

#include "planning/primitives/AnglePlanning.hpp"
#include "planning/primitives/CreatePath.hpp"
#include "planning/primitives/RRTUtil.hpp"
#include "planning/primitives/RoboCupStateSpace.hpp"

using namespace Geometry2d;
namespace Planning {

REGISTER_CONFIGURABLE(EscapeObstaclesPathPlanner);

ConfigDouble* EscapeObstaclesPathPlanner::_stepSize;
ConfigDouble* EscapeObstaclesPathPlanner::_goalChangeThreshold;

void EscapeObstaclesPathPlanner::createConfiguration(Configuration* cfg) {
    _stepSize = new ConfigDouble(
        cfg, "PathPlanner/EscapeObstaclesPathPlanner/stepSize", 0.1);
    _goalChangeThreshold = new ConfigDouble(
        cfg, "PathPlanner/EscapeObstaclesPathPlanner/goalChangeThreshold",
        Robot_Radius);
}

Trajectory EscapeObstaclesPathPlanner::plan(const PlanRequest& planRequest) {
    EASY_BLOCK("EsacpeObstaclesPathPlanner", profiler::colors::Cyan)
    const RobotInstant& startInstant = planRequest.start;
    const auto& motionConstraints = planRequest.constraints.mot;

    Geometry2d::ShapeSet obstacles;
    FillObstacles(planRequest, &obstacles, nullptr, false, nullptr);

    if (!obstacles.hit(startInstant.position()) ||
        std::holds_alternative<EmptyCommand>(planRequest.motionCommand)) {
        // Keep moving, but slow down the current velocity. This allows us to
        // keep continuity when we have short disruptions in planners (i.e.
        // single frame delay).
        // TODO(#1464): When the assignment delay is fixed, remove this horrible
        // hack by using Twist::Zero() instead of startInstant.velocity * 0.8
        Trajectory result{
            {RobotInstant{startInstant.pose, startInstant.velocity * 0.8,
                          startInstant.stamp}}};
        result.mark_angles_valid();
        result.stamp(RJ::now());
        result.setDebugText("[ESCAPE " +
                            std::to_string(planRequest.motionCommand.index()) +
                            "]");
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
