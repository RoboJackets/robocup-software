#include "plan_request.hpp"

namespace planning {

void calc_their_robot_obs(const RobotState& robot, rj_geometry::Point& obs_center,
                          double& obs_radius) {
    // TODO(Kevin): pass shared ptr instead, don't * it
    double vel_mag = robot.velocity.linear().mag();
    rj_geometry::Point unit_vel = robot.velocity.linear().normalized(
        kRobotRadius);  // put velocity in terms of robot radiuses
    obs_center = robot.pose.position() + (unit_vel * vel_mag * 0.2);
    double safety_margin = 0.9 + vel_mag * 0.2;
    obs_radius = kRobotRadius * safety_margin;
}

void fill_obstacles(const PlanRequest& in, rj_geometry::ShapeSet* out_static,
                    std::vector<DynamicObstacle>* out_dynamic, bool avoid_ball,
                    Trajectory* out_ball_trajectory) {
    out_static->clear();
    out_static->add(in.field_obstacles);
    out_static->add(in.virtual_obstacles);

    for (int shell = 0; shell < kNumShells; shell++) {
        const RobotState& their_robot = in.world_state->their_robots.at(shell);

        if (their_robot.visible) {
            rj_geometry::Point obs_center;
            double obs_radius;

            calc_their_robot_obs(their_robot, obs_center, obs_radius);
            out_static->add(std::make_shared<rj_geometry::Circle>(obs_center, obs_radius));
        }
    }

    // Add our robots, either static or dynamic depending on whether they have
    // already been planned. Account for safety margin like above.
    for (int shell = 0; shell < kNumShells; shell++) {
        const auto& robot = in.world_state->our_robots.at(shell);
        if (!robot.visible || shell == in.shell_id) {
            continue;
        }

        // TODO(Kevin): give the option to pass nullptr for obs_center to just get the radius for
        // dyn obstacles
        double vel_mag = robot.velocity.linear().mag();
        double safety_margin = 0.9 + vel_mag * 0.2;

        if (out_dynamic != nullptr && in.planned_trajectories.at(shell) != nullptr) {
            // Dynamic obstacle
            out_dynamic->emplace_back(kRobotRadius * safety_margin,
                                      in.planned_trajectories.at(shell));
        } else {
            // Static obstacle
            out_static->add(std::make_shared<rj_geometry::Circle>(robot.pose.position(),
                                                                  kRobotRadius * safety_margin));
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
