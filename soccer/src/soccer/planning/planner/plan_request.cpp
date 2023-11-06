#include "plan_request.hpp"

namespace planning {

void fill_robot_obstacle(const RobotState& robot, rj_geometry::Point& obs_center,
                         double& obs_radius) {
    // params for obstacle shift
    double obs_center_shift = 0.5;
    double obs_radius_inflation = 1.0;

    // shift obs center off robot
    obs_center =
        robot.pose.position() + (robot.velocity.linear() * kRobotRadius * obs_center_shift);

    // inflate obs radius if needed
    double vel_mag = robot.velocity.linear().mag();
    double safety_margin = vel_mag * obs_radius_inflation;
    obs_radius = kRobotRadius + (kRobotRadius * safety_margin);
}

void fill_obstacles(const PlanRequest& in, rj_geometry::ShapeSet* out_static,
                    std::vector<DynamicObstacle>* out_dynamic, bool avoid_ball,
                    Trajectory* out_ball_trajectory) {
    out_static->clear();
    out_static->add(in.field_obstacles);
    out_static->add(in.virtual_obstacles);

    rj_geometry::Point obs_center{0.0, 0.0};
    double obs_radius{1.0};

    // Add their robots as static obstacles (inflated based on velocity).
    // See calc_static_robot_obs() docstring for more info.
    for (size_t shell = 0; shell < kNumShells; shell++) {
        const RobotState& their_robot = in.world_state->their_robots.at(shell);
        fill_robot_obstacle(their_robot, obs_center, obs_radius);

        if (their_robot.visible) {
            out_static->add(std::make_shared<rj_geometry::Circle>(obs_center, obs_radius));
        }
    }

    // Add our robots, either static or dynamic depending on whether they have
    // already been planned. In both cases, radius is based on velocity like
    // above for opp robots.
    // TODO: reenable dynamic obstacles for our robots (currently
    // TrajectoryCollection is never filled at planner level)
    for (size_t shell = 0; shell < kNumShells; shell++) {
        const auto& our_robot = in.world_state->our_robots.at(shell);
        if (!our_robot.visible || shell == in.shell_id) {
            continue;
        }

        /* const Trajectory* ptr_to_traj = std::get<0>(in.planned_trajectories->get(shell)).get();
         */
        /* if (out_dynamic != nullptr && ptr_to_traj != nullptr) { */
        /*     // Dynamic obstacle */
        /*     out_dynamic->emplace_back(obs_radius, ptr_to_traj); */
        /* } else { */
        /*     // Static obstacle */
        /*     out_static->add(std::make_shared<rj_geometry::Circle>(obs_center, obs_radius)); */
        /* } */

        // Static obstacle
        fill_robot_obstacle(our_robot, obs_center, obs_radius);
        out_static->add(std::make_shared<rj_geometry::Circle>(obs_center, obs_radius));
    }

    // Adding ball as a static obstacle (because dynamic obstacles are not working)
    double ball_vel = in.world_state->ball.velocity.mag();
    out_static->add(std::make_shared<rj_geometry::Circle>(
        in.world_state->ball.position, kBallRadius * (1.0 + ball_vel) + in.min_dist_from_ball));

    // Finally, add the ball as a dynamic obstacle.
    // (This is for when the other team is trying to do ball placement, so we
    // don't interfere with them.)
    if (avoid_ball && out_dynamic != nullptr && out_ball_trajectory != nullptr) {
        // Where should we store the ball trajectory?
        *out_ball_trajectory = in.world_state->ball.make_trajectory();
        double radius = kBallRadius * (1.0 + ball_vel) + in.min_dist_from_ball;

        out_dynamic->emplace_back(radius, out_ball_trajectory);

        if (in.debug_drawer != nullptr) {
            QColor draw_color = Qt::red;
            in.debug_drawer->draw_circle(
                rj_geometry::Circle(in.world_state->ball.position, static_cast<float>(radius)),
                draw_color);
        }
    }
}

}  // namespace planning
