#include <rj_geometry/shape.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/circle.hpp>
#include <rj_geometry/stadium_shape.hpp>
#include <rj_common/planning/robot_constraints.hpp>

#include <rj_common/context.hpp>

namespace planning {

class Obstacle {
public:
    rj_geometry::Circle obstacle;
    rj_geometry::StadiumShape padding;

    //Obstacle(rj_geometry::Shape obstacle, rj_geometry::Shape padding, rj_geometry::Point pos);

    Obstacle(rj_geometry::Point pos, rj_geometry::Point vel);

    //Obstacle(rj_geometry::Circle obstacle, rj_geometry::Circle padding, rj_geometry::Point pos, rj_geometry::Point vel);
    //Obstacle(rj_geometry::Circle obstacle, rj_geometry::Circle padding, rj_geometry::Point pos);

private:
    rj_geometry::Point pos_;
    rj_geometry::Point vel_;
    
};

}