#include "PlanRequest.hpp"

namespace Planning {

void FillObstacles(const PlanRequest& in,
                   Geometry2d::ShapeSet* out_static,
                   std::vector<DynamicObstacle>* out_dynamic,
                   bool avoid_ball,
                   Trajectory* out_ball_trajectory) {
    out_static->clear();
    out_static->add(in.field_obstacles);
    out_static->add(in.virtual_obstacles);

    // Add opponent robots as static obstacles.
    for (int shell = 0; shell < Num_Shells; shell++) {
        const auto& robot = in.world_state->their_robots.at(shell);
        if (robot.visible) {
            out_static->add(std::make_shared<Geometry2d::Circle>(
                    robot.pose.position(),
                    Robot_Radius
                ));
        }
    }

    // Add our robots, either static or dynamic depending on whether they have
    // already been planned.
    for (int shell = 0; shell < Num_Shells; shell++) {
        const auto& robot = in.world_state->our_robots.at(shell);
        if (!robot.visible || shell == in.shellID) {
            continue;
        }

        if (in.planned_trajectories.at(shell) != nullptr) {
            // Dynamic obstacle
            out_dynamic->emplace_back(
                Geometry2d::Circle(Geometry2d::Point(), Robot_Radius),
                in.planned_trajectories.at(shell)
            );
        } else {
            // Static obstacle
            out_static->add(std::make_shared<Geometry2d::Circle>(
                    robot.pose.position(),
                    Robot_Radius
                ));
        }
    }

    // Finally, add the ball as a dynamic obstacle.
    if (avoid_ball && out_ball_trajectory != nullptr) {
        // Where should we store the ball trajectory?
        *out_ball_trajectory = in.world_state->ball.make_trajectory();
        out_dynamic->emplace_back(
            Geometry2d::Circle(Geometry2d::Point(), Ball_Radius),
            out_ball_trajectory);
    }
}

} // namespace Planning