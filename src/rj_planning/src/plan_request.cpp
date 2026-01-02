#include "rj_planning/plan_request.hpp"

namespace planning {

void fill_obstacles(const PlanRequest& in, ObstacleSet& out_obstacles, bool avoid_ball) {
    out_obstacles.clear();

    // Convert field_obstacles (ShapeSet) to Obstacle objects
    for (const auto& shape : in.field_obstacles.shapes()) {
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

        out_obstacles.add(
            make_moving_robot_obstacle(our_robot.pose.position(), our_robot.velocity.linear()));
    }

    // Add ball as obstacle if needed
    // Only added when STOP state is enabled
    if (in.min_dist_from_ball > 0 || avoid_ball) {
        out_obstacles.add(make_ball_obstacle(in.world_state->ball.position, in.min_dist_from_ball));

        // Add ball placement track obstacle if applicable
        auto maybe_bp_point = in.play_state.ball_placement_point();
        if (maybe_bp_point.has_value() && in.play_state.is_their_restart()) {
            rj_geometry::Point bp_point = maybe_bp_point.value();
            float ball_radius = kBallRadius + kAvoidBallDistance + in.min_dist_from_ball;

            auto stadium = std::make_shared<rj_geometry::StadiumShape>(
                in.world_state->ball.position, bp_point, ball_radius);

            // For stadium, use the same shape for obstacle and padding
            out_obstacles.add(std::make_shared<Obstacle>(stadium, stadium));
        }
    }

    // Draw all obstacles for visualization
    if (in.debug_drawer != nullptr) {
        out_obstacles.draw(in.debug_drawer);
    }
}

}  // namespace planning
