#pragma once

#include <geometry2d/point.h>

#include <rrt/2dplane/PlaneStateSpace.hpp>

namespace Planning {

/**
 * Represents the robocup field for path-planning purposes.
 */
class RoboCupStateSpace : public RRT::StateSpace<geometry2d::Point> {
 public:
  RoboCupStateSpace(const FieldDimensions& dims,
                    const geometry2d::ShapeSet& obstacles)
      : _fieldDimensions(dims), _obstacles(obstacles) {}

  geometry2d::Point randomState() const {
    double x = _fieldDimensions.FloorWidth() * (drand48() - 0.5f);
    double y =
        _fieldDimensions.FloorLength() * drand48() - _fieldDimensions.Border();
    return geometry2d::Point(x, y);
  }

  double distance(const geometry2d::Point& from,
                  const geometry2d::Point& to) const {
    return from.distTo(to);
  }

  bool stateValid(const geometry2d::Point& state) const {
    // note: _obstacles contains obstacles that define the limits of the
    // field, so we shouldn't have to check separately that the point is
    // within the field boundaries.

    return !_obstacles.hit(state);
  }

  geometry2d::Point intermediateState(const geometry2d::Point& source,
                                      const geometry2d::Point& target,
                                      double stepSize) const {
    auto dir = (target - source).norm();
    return source + dir * stepSize;
  }

  geometry2d::Point intermediateState(const geometry2d::Point& source,
                                      const geometry2d::Point& target,
                                      double minStepSize,
                                      double maxStepSize) const {
    throw std::runtime_error("Adaptive stepsize control not implemented");
  }

  bool transitionValid(const geometry2d::Point& from,
                       const geometry2d::Point& to) const {
    // Ensure that @to doesn't hit any obstacles that @from doesn't. This
    // allows the RRT to start inside an obstacle, but prevents it from
    // entering a new obstacle.
    for (const auto& shape : _obstacles.shapes()) {
      if (shape->hit(geometry2d::Segment(from, to)) && !shape->hit(from))
        return false;
    }
    return true;
  }

 private:
  const geometry2d::ShapeSet& _obstacles;
  const FieldDimensions _fieldDimensions;
};

}  // namespace Planning
