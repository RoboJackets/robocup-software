#include <rj_geometry/circle.hpp>
#include <rj_geometry/stadium_shape.hpp>
#include <rj_planning/obstacle.hpp>

namespace planning {

class StadiumObstacle : public Obstacle {
public:
    /**
     * Creates a new obstacle with stadium shape padding.
     * Intended for use with obstacles in motion.
     * @param pos rj_geometry::Point representing the obstacle's position.
     * @param vel rj_geometry::Point representing the obstacle's linear velocity.
     */
    StadiumObstacle(rj_geometry::Point pos, rj_geometry::Point vel) {
        obstacle = std::make_shared<rj_geometry::Circle>(rj_geometry::Circle(pos, kRobotRadius));
        padding = std::make_shared<rj_geometry::StadiumShape>(rj_geometry::StadiumShape(
            pos, pos + vel * scaling, 1.5 * kRobotRadius + (vel.mag() * width_scaling)));
    }

private:
    static constexpr float scaling = 0.5f;
    static constexpr float width_scaling = 0.1f;
};

}  // namespace planning