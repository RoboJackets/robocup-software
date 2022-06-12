#include "plan_request.hpp"

namespace planning {

void fill_obstacles(const PlanRequest& in, rj_geometry::ShapeSet* out_static,
                    std::vector<DynamicObstacle>* out_dynamic, bool avoid_ball,
                    Trajectory* out_ball_trajectory) {
    out_static->clear();
    out_static->add(in.field_obstacles);
    out_static->add(in.virtual_obstacles);

    // Add opponent robots as circles in static_obstacles, but shift the circle off-center depending
    // on their current velocity. Also, inflate safety margin based on robot velocity. (see
    // section 2.5:
    // https://ssl.robocup.org/wp-content/uploads/2019/03/2019_ETDP_TIGERs_Mannheim.pdf)
    //
    // At 0 m/s, obstacle center = opp robot position
    // 1 m/s = shifted by 0.2 robot rad in dir of travel
    // 2 m/s = shifted by 0.4 robot rad in dir of travel
    // (linearly increases)
    //
    // At 0 m/s, safety margin = 90% robot radius
    // 1 m/s = 110%
    // 2 m/s = 130%
    // (linearly increase)
    // TODO(Kevin) separate out this math in a function above

    for (int shell = 0; shell < kNumShells; shell++) {
        const auto& robot = in.world_state->their_robots.at(shell);

        double vel_mag = robot.velocity.linear().mag();
        rj_geometry::Point unit_vel = robot.velocity.linear().normalized(
            kRobotRadius);  // put velocity in terms of robot radiuses
        rj_geometry::Point obs_center = robot.pose.position() + (unit_vel * vel_mag * 0.2);
        double safety_margin = 0.9 + vel_mag * 0.2;

        if (robot.visible) {
            out_static->add(
                std::make_shared<rj_geometry::Circle>(obs_center, kRobotRadius * safety_margin));
        }
    }

    // Add our robots, either static or dynamic depending on whether they have
    // already been planned. Account for safety margin like above.
    for (int shell = 0; shell < kNumShells; shell++) {
        const auto& robot = in.world_state->our_robots.at(shell);
        if (!robot.visible || shell == in.shell_id) {
            continue;
        }

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
