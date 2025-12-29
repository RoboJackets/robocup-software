#include <rj_planning/obstacle.hpp>

namespace planning {

bool Obstacle::obstacle_hit(rj_geometry::Point pt) { return obstacle->hit(pt); }

bool Obstacle::padding_hit(rj_geometry::Point pt) { return padding->hit(pt); }

bool Obstacle::padding_near(rj_geometry::Point pt, float thresh) {
    return padding->near_point(pt, thresh);
}

bool Obstacle::obstacle_near(rj_geometry::Point pt, float thresh) {
    return obstacle->near_point(pt, thresh);
}

std::shared_ptr<rj_geometry::Shape> Obstacle::get_obstacle() { return obstacle; }

std::shared_ptr<rj_geometry::Shape> Obstacle::get_padding() { return padding; }

std::shared_ptr<rj_geometry::ShapeSet> Obstacle::get_shapes() {
    return std::make_shared<rj_geometry::ShapeSet>(shapes);
}
}  // namespace planning