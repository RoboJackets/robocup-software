#include <rj_geometry/shape.hpp>
#include <rj_planning/obstacle.hpp>

namespace planning {

class StaticObstacle : public Obstacle {
public:
    /**
     * Creates a new Obstacle with the desired shapes for the obstacle and padding.
     * Note that if using compound shapes like StadiumShape, you should draw the shapes
     * representing the obstacle using those shapes' draw() method, not using draw_shapes()
     * on the Obstacle's shapeset.
     * @param obstacle rj_geometry::Shape representing the obstacle; can be any Shape.
     * @param padding rj_geometry::Shape representing the padding; can be any Shape.
     */
    StaticObstacle(std::shared_ptr<rj_geometry::Shape> obstacle,
                   std::shared_ptr<rj_geometry::Shape> padding)
        : Obstacle(obstacle, padding) {}
};

}  // namespace planning
