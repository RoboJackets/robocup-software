#include "create_path.hpp"

#define _USE_MATH_DEFINES
#include <spdlog/spdlog.h>

#include <rj_constants/constants.hpp>

#include "planning/primitives/rrt_util.hpp"
#include "planning/primitives/velocity_profiling.hpp"
#include "planning/trajectory_utils.hpp"

using namespace rj_geometry;

namespace planning::CreatePath {

Trajectory simple(const LinearMotionInstant& start, const LinearMotionInstant& goal,
                  const MotionConstraints& motion_constraints, RJ::Time start_time,
                  const std::vector<Point>& intermediate_points,
                  rj_drawing::RosDebugDrawer* debug_drawer) {
    // TODO(Kevin): sometimes motion planning crashes somewhere in here when
    // you put an obstacle on its goal pos, why?
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

// TODO(Kevin) change name to "hybrid"
Trajectory rrt(const LinearMotionInstant& start, const LinearMotionInstant& goal,
               const MotionConstraints& motion_constraints, RJ::Time start_time,
               const ShapeSet& static_obstacles,
               const std::vector<DynamicObstacle>& dynamic_obstacles,
               const std::vector<Point>& bias_waypoints, rj_drawing::RosDebugDrawer* debug_drawer) {
    // if at goal, return current pose as trajectory
    if (start.position.dist_to(goal.position) < 1e-6) {
        return Trajectory{{RobotInstant{Pose(start.position, 0), Twist(), start_time}}};
    }

    // maybe we don't need an RRT
    Trajectory straight_trajectory =
        CreatePath::simple(start, goal, motion_constraints, start_time);

    RJ::Time hit_time = RJ::Time::max();
    rj_geometry::Point hit_pt;

    bool static_obs_collision_found = trajectory_hits_static(straight_trajectory, static_obstacles,
                                                             start_time, &hit_time, &hit_pt);

    if (static_obs_collision_found) {
        SPDLOG_INFO("static obstacle collision found");
    }
    if (trajectory_hits_dynamic(straight_trajectory, dynamic_obstacles, start_time, nullptr,
                                nullptr)) {
        SPDLOG_INFO("dynamic obstacle collision found");
    }

    // If we are very close to the goal (i.e. there physically can't be a robot
    // in our way) or the straight trajectory is feasible, we can use it.
    if (start.position.dist_to(goal.position) < kRobotRadius ||
        (!static_obs_collision_found &&
         !trajectory_hits_dynamic(straight_trajectory, dynamic_obstacles, start_time, nullptr,
                                  nullptr))) {
        // not an error but this makes it stand out from my other debug output
        SPDLOG_ERROR("\nRETURNING STRAIGHT TRAJ\n");
        return straight_trajectory;
    }

    // fill in points to define path, geometrically
    std::vector<Point> path_points;
    ShapeSet obstacles = static_obstacles;

    // check if robot is not right on an obstacle
    if (start.position.dist_to(hit_pt) > 2.0 * kRobotRadius) {
        // then, iteratively create a two-line-segment path around the
        // first obstacle, with a certain STEP_SIZE and MAX_ITERATIONS
        // and greedily pick first traj that works (in a given max range)
        SPDLOG_INFO("ITERATIVE SEARCH");

        path_points.push_back(start.position);
        Point dir = (hit_pt - start.position).normalized();

        // TODO(Kevin): ros param this
        float STEP_SIZE = kRobotRadius;
        Point ccw_offset = dir.perp_ccw().normalized(STEP_SIZE);
        Point cw_offset = dir.perp_cw().normalized(STEP_SIZE);

        // if we can't find a piecewise trajectory X robot widths away, give up and use RRT
        // TODO(Kevin): ros param this
        int MAX_ITERATIONS = (10 * kRobotRadius) / STEP_SIZE;

        // TODO(Kevin): why debug_drawer not always non-null??
        if (debug_drawer != nullptr) {
            // show the max bounds of the iterative search
            Point ccw_max = hit_pt + MAX_ITERATIONS * ccw_offset;
            Point cw_max = hit_pt + MAX_ITERATIONS * cw_offset;
            debug_drawer->draw_segment(Segment(ccw_max, cw_max));
        }

        // setup data structures for iterative search
        std::vector<Point> intermediate_points;
        intermediate_points.push_back(hit_pt);
        Trajectory maybe_traj;
        Point points_to_check[] = {hit_pt, hit_pt};

        // iteratively search as described above
        for (int i = 0; i < MAX_ITERATIONS; i++) {
            points_to_check[0] = hit_pt + i * ccw_offset;
            points_to_check[1] = hit_pt + i * cw_offset;

            for (int j = 0; j < 2; j++) {
                Point pt = points_to_check[j];
                intermediate_points[0] = pt;
                maybe_traj = CreatePath::simple(start, goal, motion_constraints, start_time,
                                                intermediate_points);

                bool static_obs_collision_found =
                    trajectory_hits_static(straight_trajectory, static_obstacles, start_time);
                if (!static_obs_collision_found) {
                    // TODO(Kevin): dynamic obstacles need to be accounted for
                    // return first path that works
                    return maybe_traj;
                }
            }
        }
    }

    // if iterative search can't find a good path, use RRT
    SPDLOG_INFO("FULL RRT AGAIN");
    path_points = generate_rrt(start.position, goal.position, obstacles, bias_waypoints);

    // fill in velocities along path to get Trajectory
    Trajectory path{{}};
    constexpr int kAttemptsToAvoidDynamics = 10;
    for (int i = 0; i < kAttemptsToAvoidDynamics; i++) {
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
