#pragma once

#include "Point.hpp"
#include <stdexcept>

namespace Geometry2d {

class Segment;

/**
 * The shape class provides the interface to all shapes that are subclasses.
 *
 * We expose this class and its subclasses to python through boost python, which
 * unfortunately handles abstract base classes very strangely.  The only way I
 * could get inheritance to work properly with boost was to provide
 * implementations for the methods in this class even though we'd prefer them to
 * be pure virtual (not implemented in this class).
 */
class Shape {
public:
    Shape() = default;
    virtual ~Shape() = default;
    Shape(const Shape& other) = default;
    Shape(Shape&& other) = default;
    Shape& operator=(const Shape& other) = default;
    Shape& operator=(Shape&& other) = default;

    [[nodiscard]] virtual Shape* clone() const {
        throw std::runtime_error("Unimplemented method");
    }

    [[nodiscard]] virtual bool contains_point(Point /*pt*/) const {
        throw std::runtime_error("Unimplemented method");
    }

    // TODO(1517): Refactor hit so that it doesn't force implementations to
    // have a dependency on RobotRadius in rj_constants
    /// Returns true if the given point is within one robot radius of the shape
    [[nodiscard]] virtual bool hit(Point /*pt*/) const {
        throw std::runtime_error("Unimplemented method");
    }

    [[nodiscard]] virtual bool hit(const Segment& /*seg*/) const {
        throw std::runtime_error("Unimplemented method");
    }

    [[nodiscard]] virtual bool near_point(Point /*other*/,
                                         float /*threshold*/) const {
        throw std::runtime_error("Unimplemented method");
    }

    virtual std::string to_string() { return "Shape"; }

    friend std::ostream& operator<<(std::ostream& stream, Shape& shape) {
        stream << shape.to_string();
        return stream;
    }
};

}  // namespace Geometry2d
