#include <rj_geometry/shape.hpp>
#include <rj_planning/obstacle.hpp>

namespace planning
{
class StaticObstacle : Obstacle {
public:
    StaticObstacle(std::shared_ptr<rj_geometry::Shape> obstacle,
                   std::shared_ptr<rj_geometry::Shape> padding);

}

} // namespace planning
