#pragma once

#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "point.hpp"

namespace rj_geometry {

class Segment;

/**
 * Shape is a general virtual interface for orientable polygons.
 */
class Shape {
public:
    /**
     * Returns True if the given point lies within the shape (boundary-inclusive).
     */
    [[nodiscard]] virtual bool contains_point(const Point& p) const = 0;

    /**
     * Returns True if the given point is in the shape or
     * within the specified L2 clearance region around the shape.
     *
     * Identical to intersects_point if clearance = 0.
     *
     * Sample usage:
     *  - set p to the robot position
     *  - set clearance to be robot's radius
     *  - returns whether the robot would collide with Shape at that position
     */
    [[nodiscard]] virtual bool near_point(const Point& p, float clearance) const = 0;
    // previously called hit ^ (a note for myself when refactoring, if you see this later, delete)

    /**
     * Returns True if any part of the given segment lies within the shape (boundary-inclusive).
     */
    [[nodiscard]] virtual bool intersects_segment(const Segment& seg) const = 0;

    /**
     * Returns True if the given segment is in the shape or
     * within the specified L2 clearance region around the shape.
     *
     * Identical to intersects_segment if clearance = 0.
     *
     * Sample usage:
     *  - set seg to be a proposed robot path
     *  - set clearance to be robot's radius
     *  - returns whether the robot would collide with Shape anywhere on that path
     */
    [[nodiscard]] virtual bool near_segment(const Segment& seg, float clearance) const = 0;

    /**
     * Returns a vector containing intersection points between
     * the shape's boundary and the supplied segment.
     *
     * If the shape contains line segments with potentially infinite intersections,
     * this will report the start- and endpoints of the collinear region.
     */
    [[nodiscard]] virtual std::vector<Point> get_segment_intersections(const Segment& seg) const = 0;

    /**
     * Converts to a printable string.
     * Currently pure virtual because I would be mad if I got a debug message saying "Shape".
     *
     * Overrides should be of the format "Type<relevant parameters>"
     *  - Shape<2, 3>
     */
    [[nodiscard]] virtual std::string to_string() const = 0;
    friend std::ostream& operator<<(std::ostream& stream, const Shape& shape) {
        return stream << shape.to_string();
    }

    /**
     * Publicize the destructor, but not the others.
     */
    virtual ~Shape() = default;

    /**
     * Returns a new pointer to a copy of the implementing dynamic type.
     *
     *
     * This hack for covariant return polymorphism may be overkill, but here's the justification:
     *  - Normal copy constructors are not ideal due to potential slicing.
     *  - We are trying to remove all raw pointers from the stack where possible (C++17).
     *  - Covariant return is needed for external custom Shape implementations.
     *  - Covariant return allows Circle* to override Shape* if Circle implements Shape,
     *    yet it does not allow unique_ptr<Circle> to override unique_ptr<Shape>.
     *
     * My solution is motivated from https://www.fluentcpp.com/2017/09/08/make-polymorphic-copy-modern-cpp/
     * 1. `clone_impl` makes a copy in heap and gives a raw pointer.
     * 2. This function casts the raw pointer to a `unique_ptr`.
     * 3. This function is fake virtual! Implementers should "override" it,
     *    but must change the type of the unique_ptr from Shape to their type.
     * 4. This introduces a hazard, where people may forget to implement it,
          but the only hazard is missing a way to copy, which will be obvious when needed.
     */
    [[nodiscard]] std::unique_ptr<Shape> clone() const {
        return std::unique_ptr<Shape>(clone_impl());
    }

protected:
    /**
     * A heap-allocated raw pointer copy, only for usage in clone().
     * Please go read clone() before overriding.
     */
    virtual Shape* clone_impl() const = 0;

    /**
     * Hiding these to prevent slicing. Use clone() for copies.
     */
    Shape() = default;
    Shape(const Shape& other) = default;
    Shape(Shape&& other) = default;
    Shape& operator=(const Shape& other) = default;
    Shape& operator=(Shape&& other) = default;
};

}  // namespace rj_geometry
