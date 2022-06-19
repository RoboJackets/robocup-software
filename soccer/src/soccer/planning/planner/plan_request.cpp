#include "plan_request.hpp"

namespace planning {

std::shared_ptr<rj_geometry::Circle> calc_static_robot_obs(const RobotState& robot) {
    // params for obstacle shift
    double obs_center_shift = 0.5;
    double obs_radius_inflation = 0.5;

    // shift obs center off robot
    rj_geometry::Point obs_center =
        robot.pose.position() + (robot.velocity.linear() * kRobotRadius * obs_center_shift);

    // inflate obs radius if needed
    double vel_mag = robot.velocity.linear().mag();
    double safety_margin = vel_mag * obs_radius_inflation;
    double obs_radius = kRobotRadius + (kRobotRadius * safety_margin);
    return std::make_shared<rj_geometry::Circle>(obs_center, obs_radius);
}

void fill_obstacles(const PlanRequest& in, rj_geometry::ShapeSet* out_static,
                    std::vector<DynamicObstacle>* out_dynamic, bool avoid_ball,
                    Trajectory* out_ball_trajectory) {
    out_static->clear();
    out_static->add(in.field_obstacles);
    out_static->add(in.virtual_obstacles);

    // Add their robots as static obstacles (inflated based on velocity).
    // See calc_static_robot_obs() docstring for more info.
    for (int shell = 0; shell < kNumShells; shell++) {
        const RobotState& their_robot = in.world_state->their_robots.at(shell);

        if (their_robot.visible) {
            out_static->add(calc_static_robot_obs(their_robot));
        }
    }

    // Add our robots, either static or dynamic depending on whether they have
    // already been planned. In both cases, radius is based on velocity like
    // above for opp robots.
    for (int shell = 0; shell < kNumShells; shell++) {
        const auto& robot = in.world_state->our_robots.at(shell);
        if (!robot.visible || shell == in.shell_id) {
            continue;
        }

        std::shared_ptr<rj_geometry::Circle> obs_ptr = calc_static_robot_obs(robot);

        if (out_dynamic != nullptr && in.planned_trajectories.at(shell) != nullptr) {
            // Dynamic obstacle
            out_dynamic->emplace_back(obs_ptr->radius(), in.planned_trajectories.at(shell));
        } else {
            // Static obstacle
            out_static->add(obs_ptr);
        }
    }

    // Finally, add the ball as a dynamic obstacle.
    // (This is for when the other team is trying to do ball placement, so we
    // don't interfere with them.)
    if (avoid_ball && out_dynamic != nullptr && out_ball_trajectory != nullptr) {
        // Where should we store the ball trajectory?
        *out_ball_trajectory = in.world_state->ball.make_trajectory();
        out_dynamic->emplace_back(kBallRadius, out_ball_trajectory);
    }
}

}  // namespace planning
