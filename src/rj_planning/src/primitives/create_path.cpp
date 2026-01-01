#include "rj_planning/primitives/create_path.hpp"

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
               const ObstacleSet& obstacles, const std::vector<Point>& bias_waypoints) {
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
        !trajectory_hits_static(straight_trajectory, obstacles, start_time, nullptr)) {
        return straight_trajectory;
    }

    std::vector<Point> points =
        generate_rrt(start.position, goal.position, obstacles, bias_waypoints);

    BezierPath post_bezier(points, start.velocity, goal.velocity, motion_constraints);

    Trajectory path = profile_velocity(post_bezier, start.velocity.mag(), goal.velocity.mag(),
                                       motion_constraints, start_time);

    return path;
}

static std::unordered_map<uint8_t, std::tuple<double, double, double>> cached_intermediate_tuple_{};

Trajectory intermediate(const LinearMotionInstant& start, const LinearMotionInstant& goal,
                        const MotionConstraints& motion_constraints, RJ::Time start_time,
                        const ObstacleSet& obstacles, const FieldDimensions* field_dimensions,
                        unsigned int robot_id) {
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
        (!trajectory_hits_static(straight_trajectory, obstacles, start_time, nullptr))) {
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

            // Ignore out-of-bounds intermediate points
            // The offset 0.2m is chosen because the sim prevents you from moving
            // more than 0.2m away from the border lines
            if (abs(offset.x()) > field_dimensions->width() / 2 + 0.2 ||
                abs(offset.y()) > field_dimensions->length() / 2 + 0.2) {
                continue;
            }
            Trajectory trajectory =
                CreatePath::simple(start, goal, motion_constraints, start_time, {intermediate});

            // If the trajectory does not hit an obstacle, it is valid
            if ((!trajectory_hits_static(trajectory, obstacles, start_time, nullptr))) {
                auto angle = (final_inter - start.position).angle();
                cached_intermediate_tuple_[robot_id] = {abs(angle), signbit(angle) ? -1 : 1,
                                                        (final_inter - start.position).mag()};
                return trajectory;
            }
        }
    }

    // If all else fails, use rrt to ensure obstacle avoidance
    return CreatePath::rrt(start, goal, motion_constraints, start_time, obstacles);
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

        double fin_angle = goal.position.angle_to(start.position) + angle;
        double fin_length = scale;

        // Convert the distances and angles into a point
        intermediates.push_back(start.position + rj_geometry::Point{fin_length * cos(fin_angle),
                                                                    fin_length * sin(fin_angle)});
    }

    return intermediates;
}

}  // namespace planning::CreatePath
