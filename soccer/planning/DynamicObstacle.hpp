#pragma once
#include <memory>
#include "Geometry2d/Shape.hpp"

namespace Planning {
class Trajectory;
struct DynamicObstacle {
    DynamicObstacle(const Geometry2d::Circle& circle,
                    const Trajectory* path)
        : circle(circle), path(path) {}
    Geometry2d::Circle circle;
    const Trajectory* path;
};
}  // namespace Planning