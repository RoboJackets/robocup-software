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

void fill_obstacles(const PlanRequest& in, ObstacleSet& out_obstacles, bool avoid_ball) {
    out_obstacles.clear();

    // Convert virtual_obstacles (ShapeSet) to Obstacle objects
    for (const auto& shape : in.virtual_obstacles.shapes()) {
        out_obstacles.add(std::make_shared<Obstacle>(shape, shape));
    }

    // Add opponent robots as moving obstacles (velocity-inflated)
    for (size_t shell = 0; shell < kNumShells; shell++) {
        const RobotState& their_robot = in.world_state->their_robots.at(shell);

        if (their_robot.visible) {
            out_obstacles.add(make_moving_robot_obstacle(their_robot.pose.position(),
                                                          their_robot.velocity.linear()));
        }
    }

    // Add teammate robots as moving obstacles (velocity-inflated)
    for (size_t shell = 0; shell < kNumShells; shell++) {
        const auto& our_robot = in.world_state->our_robots.at(shell);
        if (!our_robot.visible || shell == in.shell_id) {
            continue;
        }

        out_obstacles.add(make_moving_robot_obstacle(our_robot.pose.position(),
                                                      our_robot.velocity.linear()));
    }

    // Add ball as obstacle if needed
    // Only added when STOP state is enabled
    if (in.min_dist_from_ball > 0 || avoid_ball) {
        out_obstacles.add(make_ball_obstacle(in.world_state->ball.position,
                                              in.min_dist_from_ball));
        float ball_radius = kBallRadius + kAvoidBallDistance + in.min_dist_from_ball;

        // Draw ball obstacle in simulator
        if (in.debug_drawer != nullptr) {
            QColor draw_color = Qt::red;
            in.debug_drawer->draw_circle(
                rj_geometry::Circle(in.world_state->ball.position, ball_radius), draw_color);
        }

        // Add ball placement track obstacle if applicable
        auto maybe_bp_point = in.play_state.ball_placement_point();
        if (maybe_bp_point.has_value() && in.play_state.is_their_restart()) {
            rj_geometry::Point bp_point = maybe_bp_point.value();

            auto stadium = std::make_shared<rj_geometry::StadiumShape>(
                in.world_state->ball.position, bp_point, ball_radius);

            // For stadium, use the same shape for obstacle and padding
            out_obstacles.add(std::make_shared<Obstacle>(stadium, stadium));

            if (in.debug_drawer != nullptr) {
                QColor draw_color = Qt::red;
                in.debug_drawer->draw_stadium(*stadium, draw_color);
            }
        }
    }
}

}  // namespace planning
