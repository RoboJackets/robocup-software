#pragma once
#include <Geometry2d/Circle.hpp>

namespace Planning {

class Trajectory;

/**
 * @brief An obstacle that will follow some predicted trajectory.
 *
 * @detail We only support circular dynamic obstacles for now.
 */
struct DynamicObstacle {
    /**
     * @brief Create a new dynamic obstacle. The trajectory provided must
     * outlive the usage of this object.
     */
    DynamicObstacle(const Geometry2d::Circle& circle, const Trajectory* path)
        : circle(circle), path(path) {}
    Geometry2d::Circle circle;
    const Trajectory* path;
};

}  // namespace Planning