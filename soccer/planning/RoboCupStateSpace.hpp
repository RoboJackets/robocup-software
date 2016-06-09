#pragma once

#include <Geometry2d/Point.hpp>
#include <rrt/2dplane/PlaneStateSpace.hpp>

namespace Planning {

/**
 * Represents the robocup field for path-planning purposes.
 */
class RoboCupStateSpace : public RRT::StateSpace<Geometry2d::Point> {
public:
    RoboCupStateSpace(const Field_Dimensions& dims,
                      const Geometry2d::ShapeSet& obstacles)
        : _fieldDimensions(dims), _obstacles(obstacles) {}

    Geometry2d::Point randomState() const {
        float x = _fieldDimensions.FloorWidth() * (drand48() - 0.5f);
        float y = _fieldDimensions.FloorLength() * drand48() -
                  _fieldDimensions.Border();
        return Geometry2d::Point(x, y);
    }

    double distance(const Geometry2d::Point& from,
                    const Geometry2d::Point& to) const {
        return from.distTo(to);
    }

    bool stateValid(const Geometry2d::Point& state) const {
        // TODO: check against field bounds
        if (_obstacles.hit(state)) return false;
        return true;
    }

    Geometry2d::Point intermediateState(const Geometry2d::Point& source,
                                        const Geometry2d::Point& target,
                                        float stepSize) const {
        auto dir = (target - source).norm();
        return source + dir * stepSize;
    }

    Geometry2d::Point intermediateState(const Geometry2d::Point& source,
                                        const Geometry2d::Point& target,
                                        float minStepSize,
                                        float maxStepSize) const {
        throw std::runtime_error("Adaptive stepsize control not implemented");
    }

    bool transitionValid(const Geometry2d::Point& from,
                         const Geometry2d::Point& to) const {
        // TODO: what if it starts out in an obstacle?
        if (_obstacles.hit(Geometry2d::Segment(from, to))) return false;
        return true;
    }

private:
    const Geometry2d::ShapeSet& _obstacles;
    const Field_Dimensions _fieldDimensions;
};

}  // Planning
