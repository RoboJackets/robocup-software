#include <rj_planning/obstacle.hpp>

namespace planning {
    // Obstacle::Obstacle(rj_geometry::Shape obstacle, rj_geometry::Shape padding, rj_geometry::Point pos): 
    //     obstacle(obstacle), 
    //     padding(padding), 
    //     pos_(pos), 
    //     vel_(rj_geometry::Point(0, 0)) { }
    
    Obstacle::Obstacle(
        rj_geometry::Point pos,
        rj_geometry::Point vel
    ):
    pos_(pos),
    vel_(vel) {
        float scaling = 0.5f;
        obstacle = rj_geometry::Circle(pos_, kRobotRadius);
        padding = rj_geometry::StadiumShape(pos_, pos_ + vel_ * scaling, kRobotRadius + (vel.mag() * scaling));
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