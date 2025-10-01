#include <rj_planning/obstacle.hpp>

namespace planning {

    Obstacle::Obstacle(std::shared_ptr<rj_geometry::Shape> obstacle, std::shared_ptr<rj_geometry::Shape> padding): 
        obstacle(obstacle), 
        padding(padding) {
            velocity = std::make_shared<rj_geometry::Point>(0, 0);
            shapes.add(obstacle);
            shapes.add(padding);
        }
    
    Obstacle::Obstacle(
        rj_geometry::Point pos,
        rj_geometry::Point vel
    )
    {
        float scaling = 0.5f;
        float width_scaling = 0.1f;
        obstacle = std::make_shared<rj_geometry::Circle>(rj_geometry::Circle(pos, kRobotRadius));
        padding = std::make_shared<rj_geometry::StadiumShape>(rj_geometry::StadiumShape(pos, pos + vel * scaling, 1.5 * kRobotRadius + (vel.mag() * width_scaling)));
        velocity = std::make_shared<rj_geometry::Point>(vel);
        shapes.add(obstacle);
        shapes.add(dynamic_pointer_cast<rj_geometry::StadiumShape>(padding)->drawshapes());
    }

    // Obstacle::Obstacle(
    //     rj_geometry::Circle obstacle,
    //     rj_geometry::Circle padding,
    //     rj_geometry::Point pos,
    //     rj_geometry::Point vel
    // ): obstacle(obstacle),
    // padding(padding),
    // pos_(pos),
    // vel_(vel) {
    // }

    // Obstacle::Obstacle(
    //     rj_geometry::Circle obstacle,
    //     rj_geometry::Circle padding,
    //     rj_geometry::Point pos
    // ): obstacle(obstacle),
    // padding(padding),
    // pos_(pos),
    // vel_(rj_geometry::Point(0, 0)) {}

}