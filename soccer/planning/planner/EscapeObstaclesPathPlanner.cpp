#include "EscapeObstaclesPathPlanner.hpp"

#include <optional>

#include <Configuration.hpp>
#include "planning/trajectory/RRTUtil.hpp"
#include "planning/trajectory/RoboCupStateSpace.hpp"
#include "planning/trajectory/PathSmoothing.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"
#include "planning/trajectory/RoboCupStateSpace.hpp"
#include <vector>

using Geometry2d::Point;
using Geometry2d::Pose;
using Geometry2d::Twist;
using Geometry2d::ShapeSet;

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

//    Trajectory EscapeObstaclesPathPlanner::plan(PlanRequest &&planRequest) {
//        const RobotInstant &startInstant = planRequest.start;
//        const auto &motionConstraints = planRequest.constraints.mot;
//        const Geometry2d::ShapeSet &obstacles = planRequest.obstacles;
//        const Trajectory &prevPath = planRequest.prevTrajectory;
//
//        std::optional<Point> optPrevPt;
//        if (!prevPath.empty()) optPrevPt = prevPath.last().pose.position();
//        const Point unblocked = findNonBlockedGoal(startInstant.pose.position(), optPrevPt, obstacles);
//
//        // reuse path if there's not a significantly better spot to target
//        if (!prevPath.empty() && unblocked == prevPath.last().pose.position()) {
//            return std::move(planRequest.prevTrajectory);
//        }
//
//        // TODO(justbuchanan): We could potentially even use a kinodynamic RRT in
//        // findNonBlockedGoal() so it takes initial velocity into account
//        double goalAngle = startInstant.pose.position().angleTo(unblocked);//todo(Ethan) verify this
//        RobotInstant goalInstant{Pose{unblocked, goalAngle}, Twist{}, RJ::now()};
//        std::vector<Point> points{startInstant.pose.position(), unblocked};
//        std::shared_ptr<RoboCupStateSpace> stateSpace = std::make_shared<RoboCupStateSpace>(Field_Dimensions::Current_Dimensions, obstacles);
//        BezierPath new_bezier(points, startInstant.velocity.linear(), goalInstant.velocity.linear(), planRequest.constraints.mot);
//        Trajectory path = ProfileVelocity(new_bezier,
//                                          startInstant.velocity.linear().mag(),
//                                          goalInstant.velocity.linear().mag(),
//                                          planRequest.constraints.mot);
//        return std::move(path);
//    }

    Point EscapeObstaclesPathPlanner::findNonBlockedGoal(
            Point goal, std::optional<Point> prevGoal, const ShapeSet &obstacles,
            int maxItr, std::function<void(const RRT::Tree<Point> &)> rrtLogger) {
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
                RRT::Node<Point> *newNode = rrt.grow();

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
