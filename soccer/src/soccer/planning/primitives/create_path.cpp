#include "create_path.hpp"

#include <rj_constants/constants.hpp>

#include "planning/primitives/rrt_util.hpp"
#include "planning/primitives/velocity_profiling.hpp"
#include "planning/trajectory_utils.hpp"

using namespace rj_geometry;

namespace planning::CreatePath {

Trajectory simple(const LinearMotionInstant& start, const LinearMotionInstant& goal,
                  const MotionConstraints& motion_constraints, RJ::Time start_time,
                  const std::vector<Point>& intermediate_points) {
    std::vector<Point> points;
    points.push_back(start.position);
    for (const Point& pt : intermediate_points) {
        points.push_back(pt);
    }
    points.push_back(goal.position);
    BezierPath bezier(points, start.velocity, goal.velocity, motion_constraints);
    Trajectory path = profile_velocity(bezier, start.velocity.mag(), goal.velocity.mag(),
                                       motion_constraints, start_time);
    return path;
}

Trajectory rrt(const LinearMotionInstant& start, const LinearMotionInstant& goal,
               const MotionConstraints& motion_constraints, RJ::Time start_time,
               const ShapeSet& static_obstacles,
               const std::vector<DynamicObstacle>& dynamic_obstacles,
               const std::vector<Point>& bias_waypoints) {
    if (start.position.dist_to(goal.position) < 1e-6) {
        return Trajectory{{RobotInstant{Pose(start.position, 0), Twist(), start_time}}};
    }
    // maybe we don't need an RRT
    Trajectory straight_trajectory =
        CreatePath::simple(start, goal, motion_constraints, start_time);

    // find static path_breaks in the straight trajectory
    // TODO: do this for dynamic path breaks
    auto static_path_breaks =
        trajectory_hits_static(straight_trajectory, static_obstacles, start_time, nullptr);
    bool static_obs_collision = (static_path_breaks.size() == 0);

    // If we are very close to the goal (i.e. there physically can't be a robot
    // in our way) or the straight trajectory is feasible, we can use it.
    if (start.position.dist_to(goal.position) < kRobotRadius ||
        (static_obs_collision && !trajectory_hits_dynamic(straight_trajectory, dynamic_obstacles,
                                                          start_time, nullptr, nullptr))) {
        return straight_trajectory;
    }

    // otherwise, construct a point trajectory, running RRT to get across the
    // path_breaks
    // TODO: do this for dynamic path breaks
    ShapeSet obstacles = static_obstacles;
    std::vector<Point> path_points;
    path_points.push_back(start.position);
    for (std::size_t i = 0; i < static_path_breaks.size(); i += 2) {
        // path breaks will be found in pairs of (enter obstacle, exit obstacle)
        auto enter_pt = static_path_breaks[i];
        auto exit_pt = static_path_breaks[i + 1];
        std::vector<Point> path_jump = generate_rrt(enter_pt, exit_pt, obstacles, bias_waypoints);

        for (const auto& pt : path_jump) {
            path_points.push_back(pt);
        }
    }
    path_points.push_back(goal.position);

    Trajectory path{{}};
    constexpr int kAttemptsToAvoidDynamics = 10;
    for (int i = 0; i < kAttemptsToAvoidDynamics; i++) {
        /* std::vector<Point> points = */
        /*     generate_rrt(start.position, goal.position, obstacles, bias_waypoints); */

        BezierPath post_bezier(path_points, start.velocity, goal.velocity, motion_constraints);

        path = profile_velocity(post_bezier, start.velocity.mag(), goal.velocity.mag(),
                                motion_constraints, start_time);

        Circle hit_circle;
        if (!trajectory_hits_dynamic(path, dynamic_obstacles, path.begin_time(), &hit_circle,
                                     nullptr)) {
            break;
        }

        // Inflate the radius slightly so we don't try going super close to
        // it and hitting it again.
        hit_circle.radius(hit_circle.radius() * 1.5f);
        obstacles.add(std::make_shared<Circle>(hit_circle));
    }

    return path;
}

}  // namespace planning::CreatePath
