#include <rj_geometry/circle.hpp>
#include <rj_geometry/stadium_shape.hpp>
#include <rj_planning/obstacle.hpp>

namespace planning {
class StadiumObstacle : Obstacle {
public:
    StadiumObstacle(rj_geometry::Point pos, rj_geometry::Point vel);
};

}  // namespace planning