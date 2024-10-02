#include "plan_request.hpp"

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
                    std::vector<DynamicObstacle>* out_dynamic, bool avoid_ball,
                    Trajectory* out_ball_trajectory) {
    out_static->clear();
    out_static->add(in.field_obstacles);
    out_static->add(in.virtual_obstacles);

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
        out_static->add(std::make_shared<rj_geometry::Circle>(make_robot_obstacle(our_robot)));
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
            // in.debug_drawer->draw_circle(ball_obs, draw_color);
        }

        out_static->add(std::make_shared<rj_geometry::Circle>(std::move(ball_obs)));

        auto maybe_bp_point = in.play_state.ball_placement_point();
        if (maybe_bp_point.has_value()) {
            rj_geometry::Point bp_point = maybe_bp_point.value();
            auto ball_obs2 = make_inflated_static_obs(bp_point, in.world_state->ball.velocity, kBallRadius + kAvoidBallDistance);
            double x1 = ball_obs.center.x();
            double y1 = ball_obs.center.y();
            double x2 = ball_obs2.center.x();
            double y2 = ball_obs2.center.y();

            double rect_x_left = x1 - (y2 - y1) / (2 * ball_obs.radius());
            double rect_y_left = y1 + (x2 - x1) / (2 * ball_obs.radius());

            double rect_x_right = x2 + (y2 - y1) / (2 * ball_obs.radius());
            double rect_y_right = y2 - (x2 - x1) / (2 * ball_obs2.radius());

            rj_geometry::Point rect_top_left{rect_x_left, rect_y_left};
            rj_geometry::Point rect_bottom_right{rect_x_right, rect_y_right};

            rj_geometry::Rect rect_obs{rect_top_left, rect_bottom_right};

            rj_geometry::CompositeShape track_obs{};
            std::shared_ptr<rj_geometry::Shape> ball_obs_ptr = std::make_shared<rj_geometry::Shape>(ball_obs);
            std::shared_ptr<rj_geometry::Shape> rect_obs_ptr = std::make_shared<rj_geometry::Shape>(rect_obs);
            std::shared_ptr<rj_geometry::Shape> ball_obs2_ptr = std::make_shared<rj_geometry::Shape>(ball_obs2);
            track_obs.add(ball_obs_ptr);
            track_obs.add(rect_obs_ptr);
            track_obs.add(ball_obs2_ptr);

            out_static->add(std::make_shared<rj_geometry::CompositeShape>(std::move(track_obs)));

            if (in.debug_drawer != nullptr) {
                QColor draw_color = Qt::red;
                in.debug_drawer->draw_circle(ball_obs, draw_color);
                in.debug_drawer->draw_rect(rect_obs, draw_color);
                in.debug_drawer->draw_circle(ball_obs2, draw_color);
            }
        }

        // replace with composite shape: 2 circles + rectangle
        // circle 1: center of ball position, radius is kBallRadius + kAvoidBallDistance
        // rectangle 1: top left of robot is (x1 - (y2 - y1)/(2 * (ball_obs.radius())), y1 + (x2 - x1)/(2 * (ball_obs.radius())))
        // bottom right of the robot is (x2 + (y2 - y1)/(2 * (ball_obs.radius())), y2 - (x2 - x1)/(2 * (ball_obs.radius())))
        // (x1, y1) is ball position, (x2, y2) is ball placement position
        // circle 2: center of ball placement position, radius is kBallRadius + kAvoidBallDistance
    }
}

}  // namespace planning
