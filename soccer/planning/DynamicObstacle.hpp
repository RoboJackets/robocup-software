#pragma once
#include "planning/trajectory/Trajectory.hpp"
#include <memory>
#include "Geometry2d/Shape.hpp"

namespace Planning {
    class Trajectory;
    struct DynamicObstacle {
        DynamicObstacle(std::shared_ptr<Geometry2d::Shape> sHape, const Trajectory* pAth): shape(sHape), path(pAth) {}
        std::shared_ptr<Geometry2d::Shape> shape;
        const Trajectory* path;
    };
}