#include "CreatePath.hpp"

#include "planning/TrajectoryUtils.hpp"
#include "planning/low_level/RRTUtil.hpp"
#include "planning/low_level/VelocityProfiling.hpp"

using namespace Geometry2d;

namespace Planning::CreatePath {

Trajectory simple(const LinearMotionInstant& start,
                  const LinearMotionInstant& goal,
                  const MotionConstraints& motionConstraints,
                  RJ::Time startTime,
                  const std::vector<Point>& intermediatePoints) {
    std::vector<Point> points;
    points.push_back(start.position);
    for (const Point& pt : intermediatePoints) {
        points.push_back(pt);
    }
    points.push_back(goal.position);
    BezierPath bezier(points, start.velocity, goal.velocity, motionConstraints);
    Trajectory path =
        ProfileVelocity(bezier, start.velocity.mag(), goal.velocity.mag(),
                        motionConstraints, startTime);
    return std::move(path);
}

Trajectory rrt(const LinearMotionInstant& start,
               const LinearMotionInstant& goal,
               const MotionConstraints& motionConstraints,
               RJ::Time startTime,
               const ShapeSet& static_obstacles,
               const std::vector<DynamicObstacle>& dynamic_obstacles,
               const std::vector<Point>& biasWaypoints) {
    if (start.position.distTo(goal.position) < 1e-6) {
        return Trajectory{
            {RobotInstant{Pose(start.position, 0), Twist(), startTime}}};
    }
    // maybe we don't need an RRT
    Trajectory straightTrajectory =
        CreatePath::simple(start, goal, motionConstraints, startTime);

    // If we are very close to the goal (i.e. there physically can't be a robot
    // in our way) or the straight trajectory is feasible, we can use it.
    if (start.position.distTo(goal.position) < Robot_Radius ||
        !TrajectoryHitsStatic(straightTrajectory,
                              static_obstacles,
                              startTime,
                              nullptr) &&
            !TrajectoryHitsDynamic(straightTrajectory,
                                   dynamic_obstacles,
                                   startTime,
                                   nullptr,
                                   nullptr)) {
        return std::move(straightTrajectory);
    }

    ShapeSet obstacles = static_obstacles;
    Trajectory path{{}};
    constexpr int attemptsToAvoidDynamics = 10;
    for (int i = 0; i < attemptsToAvoidDynamics; i++) {
        std::vector<Point> points = GenerateRRT(start.position, goal.position,
                                                obstacles, biasWaypoints);

        BezierPath postBezier(points, start.velocity, goal.velocity,
                              motionConstraints);

        path =
            ProfileVelocity(postBezier, start.velocity.mag(),
                            goal.velocity.mag(), motionConstraints, startTime);

        Circle hitCircle;
        if (!TrajectoryHitsDynamic(path, dynamic_obstacles, path.begin_time(),
                                   &hitCircle, nullptr)) {
            break;
        }

        // Inflate the radius slightly so we don't try going super close to
        // it and hitting it again.
        hitCircle.radius(hitCircle.radius() * 1.5);
        obstacles.add(std::make_shared<Circle>(hitCircle));
    }
    return std::move(path);
}

} // namespace Planning::CreatePath
