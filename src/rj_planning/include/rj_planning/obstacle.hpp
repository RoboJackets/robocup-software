#include <rj_common/context.hpp>
#include <rj_common/planning/robot_constraints.hpp>
#include <rj_geometry/circle.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/shape.hpp>
#include <rj_geometry/stadium_shape.hpp>

namespace planning {

class Obstacle {
public:
    Obstacle(std::shared_ptr<rj_geometry::Shape> obstacle,
             std::shared_ptr<rj_geometry::Shape> padding);

    virtual bool obstacle_hit(rj_geometry::Point pt);
    virtual bool padding_hit(rj_geometry::Point pt);
    virtual bool obstacle_near(rj_geometry::Point pt, float thresh);
    virtual bool padding_near(rj_geometry::Point pt, float thresh);
    virtual shared_ptr<rj_geometry::ShapeSet> get_shapes();
    virtual std::shared_ptr<rj_geometry::Shape> get_obstacle();
    virtual std::shared_ptr<rj_geometry::Shape> get_padding();
protected:
    std::shared_ptr<rj_geometry::Shape> obstacle;
    std::shared_ptr<rj_geometry::Shape> padding;
    std::shared_ptr<rj_geometry::Point> velocity;
    rj_geometry::ShapeSet shapes;
};

}  // namespace planning