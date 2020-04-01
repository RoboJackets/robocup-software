#pragma once
#include <memory>
#include "Geometry2d/Shape.hpp"

namespace Planning {
    class Trajectory;
    struct DynamicObstacle {
        DynamicObstacle(const std::shared_ptr<Geometry2d::Circle>& circ, const Trajectory* pAth): circle(circ), path(pAth) {}
        std::shared_ptr<Geometry2d::Circle> circle;
        const Trajectory* path;
    };
}