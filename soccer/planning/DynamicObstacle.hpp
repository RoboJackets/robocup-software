#pragma once
#include "planning/trajectory/Trajectory.hpp"
#include <memory>
#include "Geometry2d/Shape.hpp"

namespace Planning {
    class Trajectory;
    struct DynamicObstacle {
        //todo(Ethan) pass by address. const ref makes it error prone
        DynamicObstacle(const std::shared_ptr<Geometry2d::Circle>& circ, const Trajectory& pAth): circle(circ), path(&pAth) {}
        std::shared_ptr<Geometry2d::Circle> circle;
        const Trajectory* path;
    };
}