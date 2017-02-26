#include "EscapeObstaclesPathPlanner.hpp"
#include <Configuration.hpp>
#include "RRTUtil.hpp"
#include "RoboCupStateSpace.hpp"
#include "TrapezoidalPath.hpp"


std::unique_ptr<Path> StraightLinePathPlanner::run(PlanRequest& planRequest) {
	const MotionInstant& startInstant = planRequest.start;
    const auto& motionConstraints = planRequest.constraints.mot;
    const Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    std::unique_ptr<Path>& prevPath = planRequest.prevPath;

}

std::unique_ptr<Path> StraightLinePathPlanner::findNonBlockedGoal(
    Point goal, boost::optional<Point> prevGoal, const ShapeSet& obstacles,
    int maxItr, std::function<void(const RRT::Tree<Point>&)> rrtLogger) {
}