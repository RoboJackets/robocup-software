#include <rj_planning/stadium_obstacle.hpp>

namespace planning {
/**
 * Creates a new obstacle with stadium shape padding.
 * Intended for use with obstacles in motion.
 * @param pos rj_geometry::Point representing the obstacle's position.
 * @param vel rj_geometry::Point representing the obstacle's linear velocity.
 */
StadiumObstacle::StadiumObstacle(rj_geometry::Point pos, rj_geometry::Point vel) {
    float scaling = 0.5f;
    float width_scaling = 0.1f;
    obstacle = std::make_shared<rj_geometry::Circle>(rj_geometry::Circle(pos, kRobotRadius));
    padding = std::make_shared<rj_geometry::StadiumShape>(rj_geometry::StadiumShape(
        pos, pos + vel * scaling, 1.5 * kRobotRadius + (vel.mag() * width_scaling)));
    velocity = std::make_shared<rj_geometry::Point>(vel);
    shapes.add(obstacle);
    shapes.add(dynamic_pointer_cast<rj_geometry::StadiumShape>(padding)->drawshapes());
}
}  // namespace planning