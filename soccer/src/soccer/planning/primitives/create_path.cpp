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
    // if already on goal, no need to move
    if (start.position.dist_to(goal.position) < 1e-6) {
        return Trajectory{{RobotInstant{Pose(start.position, 0), Twist(), start_time}}};
    }

    // maybe we don't need an RRT
    Trajectory straight_trajectory =
        CreatePath::simple(start, goal, motion_constraints, start_time);

    // If we are very close to the goal (i.e. there physically can't be a robot
    // in our way) or the straight trajectory is feasible, we can use it.
    if (start.position.dist_to(goal.position) < kRobotRadius ||
        (!trajectory_hits_static(straight_trajectory, static_obstacles, start_time, nullptr) &&
         !trajectory_hits_dynamic(straight_trajectory, dynamic_obstacles, start_time, nullptr,
                                  nullptr))) {
        return straight_trajectory;
    }

    ShapeSet obstacles = static_obstacles;
    Trajectory path{{}};
    constexpr int kAttemptsToAvoidDynamics = 10;
    for (int i = 0; i < kAttemptsToAvoidDynamics; i++) {
        std::vector<Point> points =
            generate_rrt(start.position, goal.position, obstacles, bias_waypoints);

        BezierPath post_bezier(points, start.velocity, goal.velocity, motion_constraints);

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

static std::optional<std::pair<double, double>> cached_intermediate_pair_{};


Trajectory intermediate(const LinearMotionInstant& start, const LinearMotionInstant& goal,
                        const MotionConstraints& motion_constraints, RJ::Time start_time,
                        const rj_geometry::ShapeSet& static_obstacles) {
    // if already on goal, no need to move
    if (start.position.dist_to(goal.position) < 1e-6) {
        return Trajectory{{RobotInstant{Pose(start.position, 0), Twist(), start_time}}};
    }

    // maybe straight line works
    Trajectory straight_trajectory =
        CreatePath::simple(start, goal, motion_constraints, start_time);

    // If we are very close to the goal (i.e. there physically can't be a robot
    // in our way) or the straight trajectory is feasible, we can use it.
    if (start.position.dist_to(goal.position) < kRobotRadius ||
        (!trajectory_hits_static(straight_trajectory, static_obstacles, start_time, nullptr))) {
        return straight_trajectory;
    }

    // Generate list of intermediate points
    std::vector<rj_geometry::Point> intermediates = get_intermediates(start, goal);

    for (int i = 0; i < intermediate::PARAM_num_intermediates; i++) {
        rj_geometry::Point final_inter = intermediates[i];

        // Step through the path from the robot to the final intermediate point
        // and test each point on that path as an intermediate point
    for (double t = intermediate::PARAM_step_size; t < final_inter.dist_to(start.position); t += intermediate::PARAM_step_size) {
            rj_geometry::Point intermediate =
                (final_inter - start.position).normalized(t) + start.position;
            Trajectory trajectory =
                CreatePath::simple(start, goal, motion_constraints, start_time, {intermediate});

            // If the trajectory does not hit an obstacle, it is valid
            if ((!trajectory_hits_static(trajectory, static_obstacles, start_time, nullptr))) {
                cached_intermediate_pair_ = {(final_inter - start.position).angle(), (final_inter - start.position).mag()};
                return trajectory;
            }
        }
    }

    // If all else fails, return the straight-line trajectory
    return straight_trajectory;
}

std::vector<rj_geometry::Point> get_intermediates(const LinearMotionInstant& start,
                                                  const LinearMotionInstant& goal) {
    std::random_device rd;
    std::mt19937 gen(rd());
    // Create a random distribution for the distance between the start
    // and the intermediate points
    std::uniform_real_distribution<> scale_dist(intermediate::PARAM_min_scale, intermediate::PARAM_max_scale);

    double angle_range = intermediate::PARAM_max_angle - intermediate::PARAM_min_angle;
    // Create a random distribution for the angle between the start
    // and the intermediate points
    std::uniform_real_distribution<> angle_dist(-angle_range, angle_range);

    std::vector<rj_geometry::Point> intermediates;
    std::vector<std::pair<double, double>> inter_pairs;

    for (int i = 0; i < intermediate::PARAM_num_intermediates; i++) {
        double angle = angle_dist(gen);
        angle += std::copysign(intermediate::PARAM_min_angle, angle);
        angle = degrees_to_radians(angle);
        double scale = scale_dist(gen);

        // Generate random pairs of distances and angles
        inter_pairs.emplace_back(angle, scale);
    }

    if (cached_intermediate_pair_) {
        inter_pairs.push_back(*cached_intermediate_pair_);
    }

    // Sort the list of pairs by the angle
    // This ensures that we take paths with
    // smaller offsets from the simple path
    sort(inter_pairs.begin(), inter_pairs.end());

    for (int i = 0; i < intermediate::PARAM_num_intermediates; i++) {
        double angle = inter_pairs[i].first;
        double scale = inter_pairs[i].second;

        double fin_angle = goal.position.angle_to(start.position) + angle;
        double fin_length = scale;

        // Convert the distances and angles into a point
        intermediates.push_back(start.position + rj_geometry::Point{fin_length * cos(fin_angle),
                                                                    fin_length * sin(fin_angle)});
    }

    return intermediates;
}

}  // namespace planning::CreatePath
