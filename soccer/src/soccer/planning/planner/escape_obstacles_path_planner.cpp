#include "escape_obstacles_path_planner.hpp"
#include <stddef.h>                                      // for size_t
#include <functional>                                    // for function
#include <memory>                                        // for make_shared
#include <optional>                                      // for optional
#include <rrt/Tree.hpp>                                  // for Tree, Node
#include <vector>                                        // for allocator
#include "planning/instant.hpp"                          // for RobotInstant
#include "planning/planner/plan_request.hpp"             // for PlanRequest
#include "planning/planning_params.hpp"                  // for PARAM_goal_c...
#include "planning/primitives/angle_planning.hpp"        // for plan_angles
#include "planning/primitives/create_path.hpp"           // for simple
#include "planning/primitives/robo_cup_state_space.hpp"  // for RoboCupState...
#include "planning/robot_constraints.hpp"                // for RobotConstra...
#include "rj_common/field_dimensions.hpp"                // for FieldDimensions
#include "rj_common/time.hpp"                            // for now
#include "rj_geometry/point.hpp"                         // for Point, rj_ge...
#include "rj_geometry/pose.hpp"                          // for Twist
#include "rj_geometry/shape_set.hpp"                     // for ShapeSet
using namespace rj_geometry;
namespace planning {

Trajectory EscapeObstaclesPathPlanner::plan(const PlanRequest& plan_request) {
    const RobotInstant& start_instant = plan_request.start;
    const auto& motion_constraints = plan_request.constraints.mot;

    if (!plan_request.static_obstacles.hit(start_instant.position())) {
        // Keep moving, but slow down the current velocity. This allows us to
        // keep continuity when we have short disruptions in planners (i.e.
        // single frame delay).
        Trajectory result{
            {RobotInstant{start_instant.pose, start_instant.velocity * 0.0, start_instant.stamp}}};
        result.mark_angles_valid();
        result.stamp(RJ::now());
        result.set_debug_text("[SLOW " + std::to_string(plan_request.shell_id) + "]");
        return result;
    }

    Point unblocked =
        find_non_blocked_goal(start_instant.position(), previous_target_, plan_request.static_obstacles, 300);

    std::optional<Point> opt_prev_pt;

    LinearMotionInstant goal{unblocked, Point()};
    auto result = CreatePath::simple(start_instant.linear_motion(), goal, motion_constraints,
                                     start_instant.stamp);
    plan_angles(&result, start_instant, AngleFns::tangent, plan_request.constraints.rot);
    result.set_debug_text("[ESCAPE " + std::to_string(plan_request.shell_id) + "]");

    previous_target_ = unblocked;

    result.stamp(RJ::now());
    return result;
}

Point EscapeObstaclesPathPlanner::find_non_blocked_goal(Point goal, std::optional<Point> prev_goal,
                                                        const ShapeSet& obstacles, int max_itr) {
    if (obstacles.hit(goal)) {
        auto state_space =
            std::make_shared<RoboCupStateSpace>(FieldDimensions::current_dimensions, obstacles);
        RRT::Tree<Point> rrt(state_space, Point::hash, 2);
        rrt.setStartState(goal);
        // note: we don't set goal state because we're not looking for a
        // particular point, just something that isn't blocked
        rrt.setStepSize(step_size());

        // The starting point is in an obstacle, extend the tree until we find
        // an unobstructed point
        Point new_goal;
        for (int i = 0; i < max_itr; ++i) {
            // extend towards a random point
            RRT::Node<Point>* new_node = rrt.grow();

            // if the new point is not blocked, it becomes the new goal
            if (new_node && !obstacles.hit(new_node->state())) {
                new_goal = new_node->state();
                break;
            }
        }

        if (!prev_goal || obstacles.hit(*prev_goal)) return new_goal;

        // Only use this newly-found point if it's closer to the desired goal by
        // at least a certain threshold
        float old_dist = (*prev_goal - goal).mag();
        float new_dist = (new_goal - goal).mag();
        if (new_dist + escape::PARAM_goal_change_threshold < old_dist) {
            return new_goal;
        } else {
            return *prev_goal;
        }
    }

    return goal;
}

bool EscapeObstaclesPathPlanner::is_done() const {
    // Since this is the lowest priority planner, PlannerForRobot automatically
    // switches to a more suitable planner when needed.
    // (see planner_node.cpp)
    return false;
}

}  // namespace planning
