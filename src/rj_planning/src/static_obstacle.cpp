#include <rj_planning/static_obstacle.hpp>

namespace planning {
/**
 * Creates a new Obstacle with the desired shapes for the obstacle and padding.
 * Note that if using compound shapes like StadiumShape, you should draw the shapes
 * representing the obstacle using those shapes' draw() method, not using draw_shapes()
 * on the Obstacle's shapeset.
 * @param obstacle rj_geometry::Shape representing the obstacle; can be any Shape.
 * @param padding rj_geometry::Shape representing the padding; can be any Shape.
 */
StaticObstacle::StaticObstacle(std::shared_ptr<rj_geometry::Shape> obstacle,
                               std::shared_ptr<rj_geometry::Shape> padding) {
    obstacle = obstacle;
    padding = padding;
    velocity = std::make_shared<rj_geometry::Point>(0, 0);
    shapes.add(obstacle);
    shapes.add(padding);
}

}  // namespace planning