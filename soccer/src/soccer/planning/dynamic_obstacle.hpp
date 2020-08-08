#pragma once
#include <rj_geometry/circle.hpp>

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
    DynamicObstacle(double radius, const Trajectory* path)
        : circle(rj_geometry::Point(), static_cast<float>(radius)), path(path) {}
    rj_geometry::Circle circle;
    const Trajectory* path;
};

}  // namespace Planning