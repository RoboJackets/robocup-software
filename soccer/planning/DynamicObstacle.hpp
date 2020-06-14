#pragma once
#include <Geometry2d/Circle.hpp>
#include <memory>

namespace Planning {
class Trajectory;
struct DynamicObstacle {
    DynamicObstacle(const Geometry2d::Circle& circle, const Trajectory* path)
        : circle(circle), path(path) {}
    Geometry2d::Circle circle;
    const Trajectory* path;
};
}  // namespace Planning