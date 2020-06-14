#pragma once

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Circle.hpp>
#include "planning/MotionInstant.hpp"
#include "Utils.hpp"
#include "Geometry2d/Shape.hpp"

namespace Planning {

class Path;

/*
 * This is a superclass for different MotionCommands.
 * Currently implemented are PathTarget, WorldVel, Pivot, DirectPathtarget, None
 */
class DynamicObstacle {
private:
    const Path* const path;
    const float radius;
    const Geometry2d::Point staticPoint;
    const std::shared_ptr<Geometry2d::Circle> staticObstacle;

public:
    DynamicObstacle(Geometry2d::Point staticPoint, float radius,
                    const Path* path = nullptr)
        : staticPoint(staticPoint),
          path(path),
          radius(radius),
          staticObstacle(
              std::make_shared<Geometry2d::Circle>(staticPoint, radius)) {}

    DynamicObstacle(Geometry2d::Circle circle)
        : staticPoint(circle.center),
          path(nullptr),
          radius(circle.radius()),
          staticObstacle(std::make_shared<Geometry2d::Circle>(circle)) {}

    DynamicObstacle(const Path* path, float radius);

    virtual ~DynamicObstacle() = default;

    bool hasPath() const { return path != nullptr; }

    const Path* getPath() const { return path; }

    // Radius = radius of obstacle
    float getRadius() const { return radius; }

    std::shared_ptr<Geometry2d::Circle> getStaticObstacle() const {
        return staticObstacle;
    }
};
}  // namespace Planning
