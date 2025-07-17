#include "create_path.hpp"

#include <rj_constants/constants.hpp>

#include "planning/primitives/rrt_util.hpp"
#include "planning/primitives/velocity_profiling.hpp"
#include "planning/trajectory_utils.hpp"

using namespace rj_geometry;

namespace {

// 2‑D dot for rj_geometry::Point
inline double dot(const rj_geometry::Point& a, const rj_geometry::Point& b) {
    return a.x() * b.x() + a.y() * b.y();
}

// Unit dir start→target (returns length; dir_out=(0,0) if degenerate)
inline double dir_to(const rj_geometry::Point& start,
                     const rj_geometry::Point& target,
                     rj_geometry::Point* dir_out) {
    rj_geometry::Point d = target - start;
    double m = d.mag();
    if (m < 1e-6) {
        *dir_out = {0, 0};
        return 0.0;
    }
    *dir_out = d / m;
    return m;
}

}  // namespace


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
    // pick first target the path heads toward
    const rj_geometry::Point& first_target =
        intermediate_points.empty() ? goal.position : intermediate_points.front();

    // forward dir
    rj_geometry::Point dir;
    dir_to(start.position, first_target, &dir);

    // shape tangents
    rj_geometry::Point start_vel_shape = dir * std::max(0.0, dot(start.velocity, dir));
    rj_geometry::Point goal_vel_shape{0, 0};  // stop at goal

    BezierPath bezier(points, start_vel_shape, goal_vel_shape, motion_constraints);

    // time profile: keep *actual* start speed magnitude so replans don't force stop.
    // (If you prefer forward-only speed, swap to std::max(0.0, dot(start.velocity, dir)).)
    double start_speed = start.velocity.mag();
    double goal_speed = 0.0;

    Trajectory path = profile_velocity(bezier, start_speed, goal_speed,
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

        // ensure at least start+goal
        if (points.size() < 2) {
            points = {start.position, goal.position};
        }

        // drop first waypoint if it's behind start relative to goal
        if (points.size() > 2) {
            if (dot(points[1] - start.position, goal.position - start.position) <= 0) {
                points.erase(points.begin() + 1);
            }
        }

        // forward dir start→first segment (or goal)
        const rj_geometry::Point& first_target = (points.size() > 1) ? points[1] : goal.position;
        rj_geometry::Point dir;
        dir_to(start.position, first_target, &dir);

        // tangents
        rj_geometry::Point start_vel_shape = dir * std::max(0.0, dot(start.velocity, dir));
        rj_geometry::Point goal_vel_shape{0, 0};  // stop

        BezierPath post_bezier(points, start_vel_shape, goal_vel_shape, motion_constraints);

        // profile (keep actual start speed mag; goal zero)
        double start_speed = start.velocity.mag();
        double goal_speed = 0.0;
        path = profile_velocity(post_bezier, start_speed, goal_speed,
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

static std::unordered_map<uint8_t, std::tuple<double, double, double>> cached_intermediate_tuple_{};

Trajectory intermediate(const LinearMotionInstant& start, const LinearMotionInstant& goal,
                        const MotionConstraints& motion_constraints, RJ::Time start_time,
                        const rj_geometry::ShapeSet& static_obstacles,
                        const std::vector<DynamicObstacle>& dynamic_obstacles,
                        const FieldDimensions* field_dimensions, unsigned int robot_id) {
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
    std::vector<rj_geometry::Point> intermediates = get_intermediates(start, goal, robot_id);

    for (int i = 0; i < intermediate::PARAM_num_intermediates; i++) {
        rj_geometry::Point final_inter = intermediates[i];

        // Step through the path from the robot to the final intermediate point
        // and test each point on that path as an intermediate point
        for (double t = intermediate::PARAM_step_size; t < final_inter.dist_to(start.position);
             t += intermediate::PARAM_step_size) {
            


            rj_geometry::Point intermediate =
                (final_inter - start.position).normalized(t) + start.position;
            auto offset = intermediate - field_dimensions->center_point();

            // reject if behind start relative to goal
            if (dot(intermediate - start.position, goal.position - start.position) <= 0) {
                continue;
            }
            // Ignore out-of-bounds intermediate points
            // The offset 0.2m is chosen because the sim prevents you from moving
            // more than 0.2m away from the border lines
            if (abs(offset.x()) > field_dimensions->width() / 2 + 0.2 ||
                abs(offset.y()) > field_dimensions->length() / 2 + 0.2) {
                continue;
            }
            Trajectory trajectory =
                CreatePath::simple(start, goal, motion_constraints, start_time, {intermediate});

            bool static_hit =
                trajectory_hits_static(trajectory, static_obstacles, start_time, nullptr);

            constexpr float kDynInflation = 1.2f;  // 20% bigger
            bool dynamic_hit =
                trajectory_hits_dynamic(trajectory, dynamic_obstacles,
                                        trajectory.begin_time(),  // or start_time; either ok
                                        nullptr, nullptr);

            // Accept only if no static *and* no dynamic collisions.
            if (!static_hit && !dynamic_hit) {
                auto angle = (final_inter - start.position).angle();
                cached_intermediate_tuple_[robot_id] = {abs(angle), signbit(angle) ? -1 : 1,
                                                        (final_inter - start.position).mag()};
                return trajectory;
            }

        }
    }

    // If all else fails, use rrt to ensure obstacle avoidance
    return CreatePath::rrt(start, goal, motion_constraints, start_time, static_obstacles,
                           dynamic_obstacles);
}

std::vector<rj_geometry::Point> get_intermediates(const LinearMotionInstant& start,
                                                  const LinearMotionInstant& goal,
                                                  unsigned int robot_id) {
    std::random_device rd;
    std::mt19937 gen(rd());
    // Create a random distribution for the distance between the start
    // and the intermediate points
    std::uniform_real_distribution<> scale_dist(intermediate::PARAM_min_scale,
                                                intermediate::PARAM_max_scale);

    double angle_range = intermediate::PARAM_max_angle - intermediate::PARAM_min_angle;
    // Create a random distribution for the angle between the start
    // and the intermediate points
    std::uniform_real_distribution<> angle_dist(-angle_range, angle_range);

    std::vector<rj_geometry::Point> intermediates;
    std::vector<std::tuple<double, double, double>> inter_tuples;

    for (int i = 0; i < intermediate::PARAM_num_intermediates; i++) {
        double angle = angle_dist(gen);
        angle += std::copysign(intermediate::PARAM_min_angle, angle);
        angle = degrees_to_radians(angle);
        double scale = scale_dist(gen);

        // Generate random tuples of distances and angles
        inter_tuples.emplace_back(abs(angle), signbit(angle) ? -1 : 1, scale);
    }


    // Sort the list of tuples by the magnitude of angle
    // This ensures that we take paths with
    // smaller offsets from the simple path
    sort(inter_tuples.begin(), inter_tuples.end());

    if (cached_intermediate_tuple_.find(robot_id) != cached_intermediate_tuple_.end()) {
        inter_tuples.emplace(inter_tuples.begin(), cached_intermediate_tuple_[robot_id]);
    }

    for (int i = 0; i < intermediate::PARAM_num_intermediates; i++) {
        double angle = std::get<0>(inter_tuples[i]) * std::get<1>(inter_tuples[i]);
        double scale = std::get<2>(inter_tuples[i]);

        double fin_angle = start.position.angle_to(start.position) + angle;
        double fin_length = scale;

        // Convert the distances and angles into a point
        intermediates.push_back(start.position + rj_geometry::Point{fin_length * cos(fin_angle),
                                                                    fin_length * sin(fin_angle)});
    }

    return intermediates;
}

}  // namespace planning::CreatePath
