#include "plan_request.hpp"

namespace planning {

void fill_obstacles(const PlanRequest& in, rj_geometry::ShapeSet* out_static,
                    std::vector<DynamicObstacle>* out_dynamic, bool avoid_ball,
                    Trajectory* out_ball_trajectory) {
    out_static->clear();
    out_static->add(in.field_obstacles);
    out_static->add(in.virtual_obstacles);

    // Add opponent robots as static obstacles.
    for (const auto& robot : in.world_state->their_robots) {
        if (robot.visible) {
            out_static->add(
                std::make_shared<rj_geometry::Circle>(robot.pose.position(), kRobotRadius));
        }
    }

    // Add our robots, either static or dynamic depending on whether they have
    // already been planned.
    for (RobotId shell = 0; shell < kNumShells; shell++) {
        const auto& robot = in.world_state->our_robots.at(shell);
        if (!robot.visible || shell == in.shell_id) {
            continue;
        }

        if (out_dynamic != nullptr && in.planned_trajectories.at(shell) != nullptr) {
            // Dynamic obstacle
            out_dynamic->emplace_back(kRobotRadius, in.planned_trajectories.at(shell));
        } else {
            // Static obstacle
            out_static->add(
                std::make_shared<rj_geometry::Circle>(robot.pose.position(), kRobotRadius));
        }
    }

    // Finally, add the ball as a dynamic obstacle.
    if (avoid_ball && out_dynamic != nullptr && out_ball_trajectory != nullptr) {
        // Where should we store the ball trajectory?
        *out_ball_trajectory = in.world_state->ball.make_trajectory();
        out_dynamic->emplace_back(kBallRadius, out_ball_trajectory);
    }
}

}  // namespace planning