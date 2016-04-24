#pragma once

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Circle.hpp>
#include <boost/optional.hpp>
#include "planning/MotionInstant.hpp"
#include "Utils.hpp"
#include "Geometry2d/Shape.hpp"

namespace Planning {

/*
 * This is a superclass for different MotionCommands.
 * Currently implemented are PathTarget, WorldVel, Pivot, DirectPathtarget, None
 */
class DynamicObstacle {

private:
    const Path * const path;
    const float radius;
    const Geometry2d::Point staticPoint;
    const std::shared_ptr<Geometry2d::Circle> staticObstacle;

public:

    DynamicObstacle(Geometry2d::Point staticPoint, float radius, const Path *path = nullptr)
            : staticPoint(staticPoint), path(path), radius(radius),
              staticObstacle(std::make_shared<Geometry2d::Circle>(staticPoint, radius)) {}

    DynamicObstacle(Geometry2d::Circle circle): staticPoint(circle.center), path(nullptr), radius(circle.radius()),
                                                staticObstacle(std::make_shared<Geometry2d::Circle>(circle)) {}

    DynamicObstacle(const Path *path, float radius): staticPoint(path->start().motion.pos), path(path), radius(radius),
                                                staticObstacle(std::make_shared<Geometry2d::Circle>(path->start().motion.pos, radius)) {}

    virtual ~DynamicObstacle() = default;

    bool hasPath() const {
        return path;
    }

    const Path* getPath() const {
        return path;
    }

    //Radius = radius of obstacle
    float getRadius() const {
        return radius;
    }

    std::shared_ptr<Geometry2d::Circle> getStaticObstacle() const {
        return staticObstacle;
    }

    //virtual std::unique_ptr<Planning::MotionCommand> clone() const = 0;

protected:
    //MotionCommand(const MotionCommand& that) = default;
    //MotionCommand(CommandType command) : commandType(command) {}

};
}  // namespace Planning
