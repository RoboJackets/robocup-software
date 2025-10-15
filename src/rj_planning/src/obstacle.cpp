#include <rj_planning/obstacle.hpp>

namespace planning {

/**
 * Creates a new Obstacle with the desired shapes for the obstacle and padding.
 * Note that if using compound shapes like StadiumShape, you should draw the shapes
 * representing the obstacle using those shapes' draw() method, not using draw_shapes()
 * on the Obstacle's shapeset.
 * @param obstacle rj_geometry::Shape representing the obstacle; can be any Shape.
 * @param padding rj_geometry::Shape representing the padding; can be any Shape.
 */
Obstacle::Obstacle(std::shared_ptr<rj_geometry::Shape> obstacle,
                   std::shared_ptr<rj_geometry::Shape> padding)
    : obstacle(obstacle), padding(padding) {
    velocity = std::make_shared<rj_geometry::Point>(0, 0);
    shapes.add(obstacle);
    shapes.add(padding);
}

bool Obstacle::obstacle_hit(rj_geometry::Point pt) {
    return obstacle->hit(pt);
}

bool Obstacle::padding_hit(rj_geometry::Point pt) {
    return padding->hit(pt);
}

bool Obstacle::padding_near(rj_geometry::Point pt, float thresh) {
    return padding->near_point(pt, thresh);
}

bool Obstacle::obstacle_near(rj_geometry::Point pt, float thresh) {
    return obstacle->near_point(pt, thresh);
}

std::shared_ptr<rj_geometry::Shape> Obstacle::get_obstacle() {
    return obstacle;
}

std::shared_ptr<rj_geometry::Shape> Obstacle::get_padding() {
    return padding;
}

std::shared_ptr<rj_geometry::ShapeSet> Obstacle::get_shapes() {
    return shapes;
}
}  // namespace planning