#include "plan_request.hpp"

#include "planning/global_state.hpp"

namespace planning {

PlanRequest::PlanRequest(const RobotIntent& intent, const GlobalState& global_state,
                         rj_drawing::RosDebugDrawer* debug_draw)
    : debug_drawer{debug_draw} {
    // Copy world state and play state from planner's global state
    this->world_state = global_state.world_state();
    this->play_state = global_state.play_state();

    // Get robot ID from intent
    this->shell_id = static_cast<unsigned int>(intent.robot_id);

    // Copy global overrides from planner's global state
    const rj_msgs::msg::GlobalOverride global_override = global_state.coach_state().global_override;
    this->min_dist_from_ball = global_override.min_dist_from_ball;

    // Use World State info to create our starting Instant
    const auto& robot = this->world_state.our_robots.at(intent.robot_id);
    this->start = RobotInstant{robot.pose, robot.velocity, robot.timestamp};

    // Copy MotionCommand and Constraints based on max robot speed
    const float max_robot_speed = global_override.max_speed;
    // Attempting to create trajectories with max speeds <= 0 crashes the planner (during RRT
    // generation)
    if (max_robot_speed == 0.0f) {
        // If coach node has speed set to 0,
        // force HALT by replacing the MotionCommand with an empty one.
        this->motion_command = MotionCommand{};
    } else if (max_robot_speed < 0.0f) {
        // If coach node has speed set to negative, assume infinity.
        // Negative numbers cause crashes, but 10 m/s is an effectively infinite limit.
        this->motion_command = intent.motion_command;
        this->constraints.mot.max_speed = 10.0f;
    } else {
        this->motion_command = intent.motion_command;
        this->constraints.mot.max_speed = max_robot_speed;
    }

    // Set dribbler speed to min of desired and limit
    const int max_dribbler_speed = global_override.max_dribbler_speed;
    this->dribbler_speed =
        std::min(static_cast<float>(intent.dribbler_speed), static_cast<float>(max_dribbler_speed));

    // Set up static obstacles
    this->static_obstacles.add(global_state.global_obstacles());
    this->static_obstacles.add(intent.local_obstacles);

    // Add defense area obstacles if we are not the goalie
    if (this->shell_id != global_state.goalie_id()) {
        this->static_obstacles.add(global_state.def_area_obstacles());
    }

    // Add their robots as static obstacles (inflated based on velocity).
    // See calc_static_robot_obs() docstring for more info.
    for (size_t shell = 0; shell < kNumShells; shell++) {
        const RobotState& their_robot = this->world_state.their_robots.at(shell);

        if (their_robot.visible) {
            this->static_obstacles.add(
                std::make_shared<rj_geometry::Circle>(create_robot_obstacle(their_robot)));
        }
    }

    // Add our robots, either static or dynamic depending on whether they have
    // already been planned. In both cases, radius is based on velocity like
    // above for opp robots.
    // TODO: reenable dynamic obstacles for our robots (currently
    // TrajectoryCollection is never filled at planner level)
    for (size_t shell = 0; shell < kNumShells; shell++) {
        const auto& our_robot = this->world_state.our_robots.at(shell);
        if (!our_robot.visible || shell == static_cast<unsigned int>(intent.robot_id)) {
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
        this->static_obstacles.add(std::make_shared<rj_geometry::Circle>(create_robot_obstacle(our_robot)));
    }

    // Finally, add the ball as a dynamic obstacle.
    // (This is for when the other team is trying to do ball placement, so we
    // don't interfere with them.)
    if (!motion_command.ignore_ball) {
        this->ball_trajectory = this->world_state.ball.make_trajectory();

        const double radius =
            kBallRadius + global_state.coach_state().global_override.min_dist_from_ball;

        this->dynamic_obstacles.emplace_back(radius, &this->ball_trajectory);

        if (this->debug_drawer) {
            this->debug_drawer->draw_circle(
                rj_geometry::Circle{this->world_state.ball.position, static_cast<float>(radius)},
                Qt::red);
        }
    }
}

rj_geometry::Circle create_robot_obstacle(const RobotState& robot) {
    // params for obstacle shift
    constexpr double obs_center_shift{0.5};
    constexpr double obs_radius_inflation{1.0};

    // shift obs center off robot
    rj_geometry::Point center{robot.pose.position() + (robot.velocity.linear() * kRobotRadius * obs_center_shift)};

    // inflate obs radius if needed
    const double vel_mag = robot.velocity.linear().mag();
    const double safety_margin = vel_mag * obs_radius_inflation;
    const double radius {kRobotRadius + (kRobotRadius * safety_margin)};

    return rj_geometry::Circle{center, static_cast<float>(radius)};
}

}  // namespace planning
