#include "plan_request.hpp"

#include "planning/global_state.hpp"

namespace planning {

PlanRequest::PlanRequest(const GlobalState& global_state, RobotIntent robot_intent,
                         rj_drawing::RosDebugDrawer* debug_drawer) {
    
    const auto* world_state = global_state.world_state();
    const auto goalie_id = global_state.goalie_id();
    
}

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

void fill_obstacles(const GlobalState& global_state, const RobotIntent& robot_intent,
                    rj_geometry::ShapeSet* out_static, std::vector<DynamicObstacle>* out_dynamic,
                    bool avoid_ball, Trajectory* out_ball_trajectory, DebugDrawer& debug_drawer) {
    out_static->clear();

    const bool is_goalie = global_state.goalie_id() == robot_intent.robot_id;
    if (!is_goalie) {
        out_static->add(global_state.def_area_obstacles());
    }

    out_static->add(robot_intent.local_obstacles);
    out_static->add(global_state.global_obstacles());

    const auto* world_state = global_state.world_state();
    rj_geometry::Point obs_center{0.0, 0.0};
    double obs_radius{1.0};

    // Add their robots as static obstacles (inflated based on velocity).
    // See calc_static_robot_obs() docstring for more info.
    for (size_t shell = 0; shell < kNumShells; shell++) {
        const RobotState& their_robot = world_state->their_robots.at(shell);
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
        const auto& our_robot = world_state->our_robots.at(shell);
        if (!our_robot.visible || shell == static_cast<unsigned int>(robot_intent.robot_id)) {
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

    // Finally, add the ball as a dynamic obstacle.
    // (This is for when the other team is trying to do ball placement, so we
    // don't interfere with them.)
    if (avoid_ball && out_dynamic != nullptr && out_ball_trajectory != nullptr) {
        // Where should we store the ball trajectory?
        *out_ball_trajectory = world_state->ball.make_trajectory();
        const double radius =
            kBallRadius + global_state.coach_state().global_override.min_dist_from_ball;

        out_dynamic->emplace_back(radius, out_ball_trajectory);

        QColor draw_color = Qt::red;
        debug_drawer.draw_circle(world_state->ball.position, static_cast<float>(radius),
                                 draw_color);
    }
}

}  // namespace planning
