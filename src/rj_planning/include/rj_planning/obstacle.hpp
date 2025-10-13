#include <rj_common/context.hpp>
#include <rj_common/planning/robot_constraints.hpp>
#include <rj_geometry/circle.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/shape.hpp>
#include <rj_geometry/stadium_shape.hpp>

namespace planning {

class Obstacle {
public:
    std::shared_ptr<rj_geometry::Shape> obstacle;
    std::shared_ptr<rj_geometry::Shape> padding;
    std::shared_ptr<rj_geometry::Point> velocity;
    rj_geometry::ShapeSet shapes;

    Obstacle(std::shared_ptr<rj_geometry::Shape> obstacle,
             std::shared_ptr<rj_geometry::Shape> padding);

    Obstacle(rj_geometry::Point pos, rj_geometry::Point vel);
};

}  // namespace planning