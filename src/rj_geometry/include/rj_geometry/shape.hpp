#pragma once

#include "point.hpp"
#include <stdexcept>

namespace rj_geometry {

class Segment;

/**
 * The shape class provides the interface to all shapes that are subclasses.
 */
class Shape {
public:
    Shape() = default;
    virtual ~Shape() = default;
    Shape(const Shape& other) = default;
    Shape(Shape&& other) = default;
    Shape& operator=(const Shape& other) = default;
    Shape& operator=(Shape&& other) = default;

    [[nodiscard]] virtual Shape* clone() const = 0;

    [[nodiscard]] virtual bool contains_point(Point) const = 0;

    // TODO(1517): Refactor hit so that it doesn't force implementations to
    // have a dependency on RobotRadius in rj_constants
    // Returns true if the given point is within one robot radius of the shape
    [[nodiscard]] virtual bool hit(Point) const = 0;

    [[nodiscard]] virtual bool hit(const Segment&) const = 0;

    [[nodiscard]] virtual bool near_point(Point, float) const = 0;

    virtual std::string to_string() { return "Shape"; }

    friend std::ostream& operator<<(std::ostream& stream, Shape& shape) {
        stream << shape.to_string();
        return stream;
    }
};

}  // namespace rj_geometry
