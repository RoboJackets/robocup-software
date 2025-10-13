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
    Obstacle::Obstacle(std::shared_ptr<rj_geometry::Shape> obstacle, std::shared_ptr<rj_geometry::Shape> padding): 
        obstacle(obstacle), 
        padding(padding) {
        velocity = std::make_shared<rj_geometry::Point>(0, 0);
        shapes.add(obstacle);
        shapes.add(padding);
    }
    
    /**
     * Creates a new obstacle with stadium shape padding.
     * Intended for use with obstacles in motion.
     * @param pos rj_geometry::Point representing the obstacle's position.
     * @param vel rj_geometry::Point representing the obstacle's linear velocity.
     */
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
}