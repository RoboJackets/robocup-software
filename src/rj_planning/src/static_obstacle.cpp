#include <rj_planning/static_obstacle.hpp>

namespace planning {
StaticObstacle::StaticObstacle(std::shared_ptr<rj_geometry::Shape> obstacle,
                               std::shared_ptr<rj_geometry::Shape> padding)
    : Obstacle(obstacle, padding) {}

}  // namespace planning