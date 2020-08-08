#pragma once

#include <rj_common/field_dimensions.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include <rrt/2dplane/PlaneStateSpace.hpp>

namespace Planning {

/**
 * Represents the robocup field for path-planning purposes.
 */
class RoboCupStateSpace : public RRT::StateSpace<Geometry2d::Point> {
public:
    RoboCupStateSpace(const FieldDimensions& dims,
                      const Geometry2d::ShapeSet& obstacles)
        : field_dimensions_(dims), obstacles_(obstacles) {}

    Geometry2d::Point randomState() const override {
        double x = field_dimensions_.floor_width() * (drand48() - 0.5f);
        double y = field_dimensions_.floor_length() * drand48() -
                   field_dimensions_.border();
        return Geometry2d::Point(x, y);
    }

    double distance(const Geometry2d::Point& from,
                    const Geometry2d::Point& to) const override {
        return from.dist_to(to);
    }

    bool stateValid(const Geometry2d::Point& state) const override {
        // note: obstacles_ contains obstacles that define the limits of the
        // field, so we shouldn't have to check separately that the point is
        // within the field boundaries.

        return !obstacles_.hit(state);
    }

    Geometry2d::Point intermediateState(const Geometry2d::Point& source,
                                        const Geometry2d::Point& target,
                                        double step_size) const override {
        auto dir = (target - source).norm();
        return source + dir * step_size;
    }

    Geometry2d::Point intermediateState(const Geometry2d::Point& source,
                                        const Geometry2d::Point& target,
                                        double min_step_size,
                                        double max_step_size) const override {
        throw std::runtime_error("Adaptive stepsize control not implemented");
    }

    bool transitionValid(const Geometry2d::Point& from,
                         const Geometry2d::Point& to) const override {
        // Ensure that @to doesn't hit any obstacles that @from doesn't. This
        // allows the RRT to start inside an obstacle, but prevents it from
        // entering a new obstacle.
        for (const auto& shape : obstacles_.shapes()) {
            if (shape->hit(Geometry2d::Segment(from, to)) && !shape->hit(from))
                return false;
        }
        return true;
    }

private:
    const Geometry2d::ShapeSet& obstacles_;
    const FieldDimensions field_dimensions_;
};

}  // namespace Planning
