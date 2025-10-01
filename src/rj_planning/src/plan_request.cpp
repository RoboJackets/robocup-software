#include "rj_planning/plan_request.hpp"

namespace planning {

rj_geometry::Circle make_inflated_static_obs(rj_geometry::Point position,
                                             rj_geometry::Point velocity, double radius) {
    // params for obstacle shift
    constexpr double obs_center_shift{0.5};
    constexpr double obs_radius_inflation{1.0};

    rj_geometry::Point obs_center{position + (velocity * radius * obs_center_shift)};

    double safety_margin{velocity.mag() * obs_radius_inflation};
    double obs_radius{radius + (safety_margin * radius)};

    return rj_geometry::Circle{obs_center, static_cast<float>(obs_radius)};
}

rj_geometry::Circle make_robot_obstacle(const RobotState& robot) {
    return make_inflated_static_obs(robot.pose.position(), robot.velocity.linear(), kRobotRadius);
}

void fill_obstacles(const PlanRequest& in, rj_geometry::ShapeSet* out_static,
                    std::vector<Obstacle>* out_dynamic, bool avoid_ball,
                    Trajectory* out_ball_trajectory) {
    out_static->clear();
    out_static->add(in.field_obstacles);
    out_static->add(in.virtual_obstacles);

    // out_dynamic->clear();
    // out_dynamic->add(in.field_obstacles);
    // out_dynamic->add(in.virtual_obstacles);

    // Add their robots as static obstacles (inflated based on velocity).
    // See calc_static_robot_obs() docstring for more info.
    for (size_t shell = 0; shell < kNumShells; shell++) {
        const RobotState& their_robot = in.world_state->their_robots.at(shell);

        if (their_robot.visible) {
            out_static->add(
                std::make_shared<rj_geometry::Circle>(make_robot_obstacle(their_robot)));
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
        // TEMPORARY: Check and see how dynamic obstacles currently work
        //  std::shared_ptr<const Trajectory> ptr_to_traj = std::get<0>(in.planned_trajectories->get(shell));
        //  float obs_radius = kRobotRadius;
        //  rj_geometry::Point obs_center = our_robot.pose.position();
        //  if (out_dynamic != nullptr && ptr_to_traj != nullptr) { 
        //      // Dynamic obstacle */
        //      out_dynamic->emplace_back(obs_radius, ptr_to_traj); 
        //  } else { 
        //      // Static obstacle */
        //      out_static->add(std::make_shared<rj_geometry::Circle>(obs_center, obs_radius)); 
        //  } 

        // Static obstacle
        out_static->add(std::make_shared<rj_geometry::Circle>(make_robot_obstacle(our_robot)));
        rj_geometry::Point obs_center = our_robot.pose.position();
        rj_geometry::Circle c = rj_geometry::Circle(obs_center, kRobotRadius);

        rj_geometry::Circle robot_padding = rj_geometry::Circle(obs_center, kRobotRadius * 2);
        if (out_dynamic != nullptr) { out_dynamic->emplace_back(our_robot.pose.position(), our_robot.velocity.linear());
            // Obstacle o{c, robot_padding, our_robot.pose.position()};
            // rj_geometry::Shape& x = o.obstacle;
            // rj_geometry::Circle& c = dynamic_cast<rj_geometry::Circle&>(x);

            // SPDLOG_INFO("Obstacle with center {} and padding radius", c.center.x());

            //SPDLOG_INFO("Obstacle with center {} and padding radius {}", static_cast<rj_geometry::Circle>(o.obstacle).center.x(), static_cast<rj_geometry::Circle>(o.obstacle).center.y(), static_cast<rj_geometry::Circle>(o.padding).radius);
            //if (out_dynamic->size() > 0) in.debug_drawer->draw_stadium(dynamic_cast<rj_geometry::StadiumShape&>(out_dynamic->at(out_dynamic->size() - 1).padding), Qt::red);
            //if (out_dynamic->size() > 0) in.debug_drawer->draw_circle((out_dynamic->at(out_dynamic->size()-1).obstacle&), Qt::red);
            //if (out_dynamic->size() > 0) in.debug_drawer->draw_circle((out_dynamic->at(out_dynamic->size() - 1).padding&), Qt::blue);
            //if (out_dynamic->size() > 0) in.debug_drawer->draw_shapes(out_dynamic->at(out_dynamic->size() - 1).shapes, Qt::blue);
        }
        //in.debug_drawer->draw_circle(static_cast<rj_geometry::Circle&>(out_dynamic->at(shell).obstacle), Qt::red);
    }

    // Adding ball as a static obstacle (because dynamic obstacles are not working)
    // Only added when STOP state is enabled
    if (in.min_dist_from_ball > 0 || avoid_ball) {
        auto ball_obs =
            make_inflated_static_obs(in.world_state->ball.position, in.world_state->ball.velocity,
                                     kBallRadius + kAvoidBallDistance);
        ball_obs.radius(ball_obs.radius() + in.min_dist_from_ball);

        // Draw ball obstacle in simulator
        if (in.debug_drawer != nullptr) {
            QColor draw_color = Qt::red;
            in.debug_drawer->draw_circle(ball_obs, draw_color);
        }

        out_static->add(std::make_shared<rj_geometry::Circle>(std::move(ball_obs)));

        auto maybe_bp_point = in.play_state.ball_placement_point();
        if (maybe_bp_point.has_value() && in.play_state.is_their_restart()) {
            rj_geometry::Point bp_point = maybe_bp_point.value();
            rj_geometry::StadiumShape stadium = rj_geometry::StadiumShape{
                in.world_state->ball.position, bp_point, ball_obs.radius()};

            std::shared_ptr<rj_geometry::StadiumShape> track_obs_ptr =
                std::make_shared<rj_geometry::StadiumShape>(stadium);

            out_static->add(track_obs_ptr);

            if (in.debug_drawer != nullptr) {
                QColor draw_color = Qt::red;
                in.debug_drawer->draw_stadium(stadium, draw_color);
            }
        }
    }
}

}  // namespace planning
