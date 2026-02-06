#include "rj_planning/primitives/create_path.hpp"

#include <algorithm>
#include <queue>
#include <unordered_map>

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

Trajectory astar(const LinearMotionInstant& start, const LinearMotionInstant& goal,
                 const MotionConstraints& motion_constraints, RJ::Time start_time,
                 const rj_geometry::ShapeSet& static_obstacles,
                 const FieldDimensions* field_dimensions) {
    // Grid-based A* with 4-directional movement (up/down/left/right).
    // Uses Euclidean distance (dist_to) as the heuristic — a generous
    // (admissible) approximation that assumes no obstacles.
    // Caps iterations to stay fast; returns empty trajectory on failure
    // so the caller can fall back to RRT.

    constexpr double kCellSize = 0.1;        // grid resolution in meters
    constexpr int kMaxIterations = 2000;      // hard cap on node expansions

    using GridCell = std::pair<int, int>;

    // ---- coordinate conversions ----
    auto to_grid = [&](const Point& pt) -> GridCell {
        return {static_cast<int>(std::round(pt.x() / kCellSize)),
                static_cast<int>(std::round(pt.y() / kCellSize))};
    };

    auto to_point = [&](const GridCell& cell) -> Point {
        return Point(cell.first * kCellSize, cell.second * kCellSize);
    };

    GridCell start_cell = to_grid(start.position);
    GridCell goal_cell = to_grid(goal.position);

    // trivial case: already on the goal cell
    if (start_cell == goal_cell) {
        return Trajectory{{RobotInstant{Pose(start.position, 0), Twist(), start_time}}};
    }

    // ---- field-bounds helper ----
    double half_width = field_dimensions->width() / 2.0 + 0.2;
    double half_length = field_dimensions->length() / 2.0 + 0.2;
    Point center = field_dimensions->center_point();

    auto in_bounds = [&](const GridCell& cell) -> bool {
        Point pt = to_point(cell);
        auto offset = pt - center;
        return std::abs(offset.x()) <= half_width && std::abs(offset.y()) <= half_length;
    };

    // ---- collision helper (uses ShapeSet::hit which inflates by kRobotRadius) ----
    auto is_free = [&](const GridCell& cell) -> bool {
        if (!in_bounds(cell)) return false;
        return !static_obstacles.hit(to_point(cell));
    };

    // bail early if start or goal cell is blocked
    if (!is_free(start_cell) || !is_free(goal_cell)) {
        return Trajectory{{}};
    }

    // ---- heuristic: straight-line distance (generous / admissible) ----
    Point goal_pt = to_point(goal_cell);
    auto heuristic = [&](const GridCell& cell) -> double {
        return to_point(cell).dist_to(goal_pt);
    };

    // ---- hash for GridCell (std::pair<int,int>) ----
    struct GridCellHash {
        size_t operator()(const GridCell& cell) const {
            size_t seed = 0;
            boost::hash_combine(seed, cell.first);
            boost::hash_combine(seed, cell.second);
            return seed;
        }
    };

    // ---- priority-queue node ----
    struct AStarNode {
        GridCell cell;
        double f;  // f = g + h
        bool operator>(const AStarNode& other) const { return f > other.f; }
    };

    // ---- A* bookkeeping ----
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
    std::unordered_map<GridCell, double, GridCellHash> g_score;
    std::unordered_map<GridCell, GridCell, GridCellHash> came_from;

    g_score[start_cell] = 0.0;
    open_set.push({start_cell, heuristic(start_cell)});

    // 4 cardinal directions: right, left, up, down
    constexpr int dx[] = {1, -1, 0, 0};
    constexpr int dy[] = {0, 0, 1, -1};

    int iterations = 0;
    bool found = false;

    while (!open_set.empty() && iterations < kMaxIterations) {
        iterations++;
        AStarNode current = open_set.top();
        open_set.pop();

        // goal reached
        if (current.cell == goal_cell) {
            found = true;
            break;
        }

        // skip stale queue entries (a better path was already found)
        double current_g = g_score.count(current.cell) ? g_score[current.cell]
                                                       : std::numeric_limits<double>::infinity();
        if (current.f > current_g + heuristic(current.cell) + 1e-9) {
            continue;
        }

        // expand 4 neighbors
        for (int i = 0; i < 4; i++) {
            GridCell neighbor = {current.cell.first + dx[i], current.cell.second + dy[i]};

            if (!is_free(neighbor)) continue;

            double tentative_g = current_g + kCellSize;

            if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
                g_score[neighbor] = tentative_g;
                came_from[neighbor] = current.cell;
                open_set.push({neighbor, tentative_g + heuristic(neighbor)});
            }
        }
    }

    if (!found) {
        return Trajectory{{}};  // exceeded iteration budget — let RRT handle it
    }

    // ---- reconstruct raw grid path (goal → start, then reverse) ----
    std::vector<Point> grid_path;
    GridCell trace = goal_cell;
    while (trace != start_cell) {
        grid_path.push_back(to_point(trace));
        trace = came_from[trace];
    }
    grid_path.push_back(to_point(start_cell));
    std::reverse(grid_path.begin(), grid_path.end());

    // ---- simplify: keep only turning points (where the cardinal direction changes) ----
    std::vector<Point> waypoints;
    waypoints.push_back(start.position);  // exact continuous start
    for (size_t i = 1; i + 1 < grid_path.size(); i++) {
        double dx1 = grid_path[i].x() - grid_path[i - 1].x();
        double dy1 = grid_path[i].y() - grid_path[i - 1].y();
        double dx2 = grid_path[i + 1].x() - grid_path[i].x();
        double dy2 = grid_path[i + 1].y() - grid_path[i].y();
        // direction changed → this is a turning point
        if (std::abs(dx1 - dx2) > 1e-9 || std::abs(dy1 - dy2) > 1e-9) {
            waypoints.push_back(grid_path[i]);
        }
    }
    waypoints.push_back(goal.position);  // exact continuous goal

    // ---- build trajectory via Bezier + velocity profiling (same as rrt/simple) ----
    BezierPath bezier(waypoints, start.velocity, goal.velocity, motion_constraints);
    Trajectory trajectory = profile_velocity(bezier, start.velocity.mag(), goal.velocity.mag(),
                                             motion_constraints, start_time);
    return trajectory;
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
            if ((!trajectory_hits_static(trajectory, static_obstacles, start_time, nullptr))) {
                auto angle = (final_inter - start.position).angle();
                cached_intermediate_tuple_[robot_id] = {abs(angle), signbit(angle) ? -1 : 1,
                                                        (final_inter - start.position).mag()};
                return trajectory;
            }
        }
    }

    // Try A* grid search — fast 4-directional planner that avoids static obstacles.
    // If it finds a collision-free trajectory, use it; otherwise fall through to RRT.
    Trajectory astar_trajectory =
        CreatePath::astar(start, goal, motion_constraints, start_time, static_obstacles,
                          field_dimensions);
    if (!astar_trajectory.empty() &&
        !trajectory_hits_static(astar_trajectory, static_obstacles, start_time, nullptr)) {
        return astar_trajectory;
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

        double fin_angle = goal.position.angle_to(start.position) + angle;
        double fin_length = scale;

        // Convert the distances and angles into a point
        intermediates.push_back(start.position + rj_geometry::Point{fin_length * cos(fin_angle),
                                                                    fin_length * sin(fin_angle)});
    }

    return intermediates;
}

}  // namespace planning::CreatePath
